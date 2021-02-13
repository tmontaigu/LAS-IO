//##########################################################################
//#                                                                        #
//#                           ExampleIOPlugin                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include <QString>
#include <QDate>

#include "LASIOFilter.h"
#include "LASOpenDialog.h"
#include "LASSaveDialog.h"
#include "LasScalarFieldLoader.h"
#include "LasScalarFieldSaver.h"

#include <GenericProgressCallback.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

#include <QtCore/QElapsedTimer>
#include <QtCore/QFileInfo>

#include <laszip/laszip_api.h>

#include <numeric>
#include <utility>

const char *LAS_METADATA_INFO_KEY = "LAS.savedInfo";

/// Holds Meta-Information about the original file that we want to save
/// to restore them when writing
struct LasSavedInfo
{
    LasSavedInfo() = default;

    explicit LasSavedInfo(const laszip_header &header)
        : fileSourceId(header.file_source_ID), guidData1(header.project_ID_GUID_data_1),
          guidData2(header.project_ID_GUID_data_2), guidData3(header.project_ID_GUID_data_3),
          versionMinor(header.version_minor), pointFormat(header.point_data_format),
          xScale(header.x_scale_factor), yScale(header.y_scale_factor), zScale(header.z_scale_factor)
    {
        strncpy(guidData4, header.project_ID_GUID_data_4, 8);
        strncpy(systemIdentifier, header.system_identifier, 32);
        const auto vlrShouldBeCopied = [](const laszip_vlr_struct &vlr) {
            return !isLaszipVlr(vlr) && !isExtraBytesVlr(vlr);
        };

        numVlrs = std::count_if(
            header.vlrs, header.vlrs + header.number_of_variable_length_records, vlrShouldBeCopied);

        if (numVlrs > 0)
        {
            vlrs = new laszip_vlr_struct[numVlrs];
            std::copy_if(
                header.vlrs, header.vlrs + header.number_of_variable_length_records, vlrs, vlrShouldBeCopied);
        }
    }

    LasSavedInfo(const LasSavedInfo &rhs)
        : fileSourceId(rhs.fileSourceId), guidData1(rhs.guidData1), guidData2(rhs.guidData2),
          guidData3(rhs.guidData3), versionMinor(rhs.versionMinor), pointFormat(rhs.pointFormat),
          xScale(rhs.xScale), yScale(rhs.yScale), zScale(rhs.zScale), numVlrs(rhs.numVlrs),
          extraScalarFields(rhs.extraScalarFields)
    {

        strncpy(guidData4, rhs.guidData4, 8);
        strncpy(systemIdentifier, rhs.systemIdentifier, 32);
        vlrs = new laszip_vlr_struct[numVlrs];
        std::copy(rhs.vlrs, rhs.vlrs + rhs.numVlrs, vlrs);
    }

    virtual ~LasSavedInfo()
    {
        delete[] vlrs;
    }

    laszip_U16 fileSourceId{};
    laszip_U32 guidData1{};
    laszip_U16 guidData2{};
    laszip_U16 guidData3{};
    laszip_CHAR guidData4[8]{};
    laszip_U8 versionMinor{};
    laszip_U8 pointFormat{};
    laszip_CHAR systemIdentifier[32]{};
    double xScale{};
    double yScale{};
    double zScale{};

    laszip_U32 numVlrs{0};
    laszip_vlr_struct *vlrs{nullptr};
    std::vector<LasExtraScalarField> extraScalarFields;
};

Q_DECLARE_METATYPE(LasSavedInfo);

static CCVector3d GetGlobalShift(FileIOFilter::LoadParameters &parameters,
                                 bool &preserveCoordinateShift,
                                 const CCVector3d &lasOffset,
                                 const CCVector3d &firstPoint)
{
    ccGlobalShiftManager::Mode csModeBackup = parameters.shiftHandlingMode;
    CCVector3d shift{};
    bool useLasOffset = false;

    // set the lasOffset as default if none was provided
    if (lasOffset.norm2() != 0 &&
        (!parameters.coordinatesShiftEnabled || !*parameters.coordinatesShiftEnabled))
    {
        if (csModeBackup !=
            ccGlobalShiftManager::NO_DIALOG) // No dialog, practically means that we don't want any shift!
        {
            useLasOffset = true;
            shift = -lasOffset;
            if (csModeBackup != ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT)
            {
                parameters.shiftHandlingMode = ccGlobalShiftManager::ALWAYS_DISPLAY_DIALOG;
            }
        }
    }

    FileIOFilter::HandleGlobalShift(firstPoint, shift, preserveCoordinateShift, parameters, useLasOffset);

    // restore previous parameters
    parameters.shiftHandlingMode = csModeBackup;
    return shift;
}

laszip_header
InitLaszipHeader(const LASSaveDialog &saveDialog, LasSavedInfo &savedInfo, ccPointCloud &pointCloud)
{
    laszip_header laszipHeader{};

    QDate currentDate = QDate::currentDate();
    laszipHeader.file_creation_year = currentDate.year();
    laszipHeader.file_creation_day = currentDate.dayOfYear();

    // TODO global encoding
    laszipHeader.version_major = 1;
    laszipHeader.version_minor = saveDialog.selectedVersionMinor();
    laszipHeader.point_data_format = saveDialog.selectedPointFormat();

    laszipHeader.header_size = HeaderSize(laszipHeader.version_minor);
    laszipHeader.offset_to_point_data = laszipHeader.header_size;
    laszipHeader.point_data_record_length = PointFormatSize(laszipHeader.point_data_format);

    CCVector3d lasScale = saveDialog.chosenScale();
    laszipHeader.x_scale_factor = lasScale.x;
    laszipHeader.y_scale_factor = lasScale.y;
    laszipHeader.z_scale_factor = lasScale.z;

    laszipHeader.file_source_ID = savedInfo.fileSourceId;
    laszipHeader.project_ID_GUID_data_1 = savedInfo.guidData1;
    laszipHeader.project_ID_GUID_data_2 = savedInfo.guidData2;
    laszipHeader.project_ID_GUID_data_3 = savedInfo.guidData3;
    strncpy(laszipHeader.project_ID_GUID_data_4, savedInfo.guidData4, 8);
    strncpy(laszipHeader.system_identifier, savedInfo.systemIdentifier, 32);
    strncpy(laszipHeader.generating_software, "CloudCompare", 32);

    if (savedInfo.extraScalarFields.empty())
    {
        // 'steal' saved vlrs
        laszipHeader.number_of_variable_length_records = savedInfo.numVlrs;
        laszipHeader.vlrs = savedInfo.vlrs;
        savedInfo.numVlrs = 0;
        savedInfo.vlrs = nullptr;
    }
    else
    {
        laszipHeader.number_of_variable_length_records = savedInfo.numVlrs + 1;
        laszipHeader.vlrs = new laszip_vlr_struct[laszipHeader.number_of_variable_length_records];
        std::copy(savedInfo.vlrs, savedInfo.vlrs + savedInfo.numVlrs, laszipHeader.vlrs);

        LasExtraScalarField::InitExtraBytesVlr(
            laszipHeader.vlrs[laszipHeader.number_of_variable_length_records - 1],
            savedInfo.extraScalarFields);
    }

    laszipHeader.offset_to_point_data +=
        SizeOfVlrs(laszipHeader.vlrs, laszipHeader.number_of_variable_length_records);

    if (pointCloud.isShifted())
    {
        laszipHeader.x_offset = -pointCloud.getGlobalShift().x;
        laszipHeader.y_offset = -pointCloud.getGlobalShift().y;
        laszipHeader.z_offset = -pointCloud.getGlobalShift().z;
    }
    else
    {
        CCVector3d bbMax;
        CCVector3d bbMin;
        pointCloud.getGlobalBB(bbMin, bbMax);
        laszipHeader.x_offset = bbMin.x;
        laszipHeader.y_offset = bbMin.y;
        laszipHeader.z_offset = bbMin.z;
    }

    unsigned int byteOffset{0};
    for (LasExtraScalarField &extraScalarField : savedInfo.extraScalarFields)
    {
        extraScalarField.byteOffset = byteOffset;
        byteOffset += extraScalarField.byteSize();
        if (extraScalarField.numElements() > 1)
        {
            // Array fields are split into multiple ccScalarField
            // and each of them has the index appended to the name
            char name[50];
            unsigned int found{0};
            for (int i = 0; i < extraScalarField.numElements(); ++i)
            {
                snprintf(name, 50, "%s [%d]", extraScalarField.name, i);
                int pos = pointCloud.getScalarFieldIndexByName(name);
                if (pos >= 0)
                {
                    extraScalarField.scalarFields[i] =
                        dynamic_cast<ccScalarField *>(pointCloud.getScalarField(pos));
                    found++;
                    ccLog::Warning("[LAS] field %s found", name);
                }
                else
                {
                    ccLog::Warning("[LAS] field %s not found", name);
                    extraScalarField.scalarFields[i] = nullptr;
                }
            }
            if (found != extraScalarField.numElements())
            {
                // TODO
                throw std::runtime_error("Not handled");
            }
        }
        else
        {
            const char *nameToSearch;
            if (!extraScalarField.ccName.empty())
            {
                // This field's name clashed with existing ccScalarField when created
                nameToSearch = extraScalarField.ccName.c_str();
            }
            else
            {
                nameToSearch = extraScalarField.name;
            }
            int pos = pointCloud.getScalarFieldIndexByName(nameToSearch);
            if (pos >= 0)
            {
                extraScalarField.scalarFields[0] =
                    dynamic_cast<ccScalarField *>(pointCloud.getScalarField(pos));
            }
            else
            {
                ccLog::Warning("[LAS] field %s not found", nameToSearch);
            }
        }
    }

    unsigned int totalExtraByteSize = LasExtraScalarField::TotalExtraBytesSize(savedInfo.extraScalarFields);
    laszipHeader.point_data_record_length += totalExtraByteSize;
    return laszipHeader;
}

LASIOFilter::LASIOFilter()
    : FileIOFilter({"LAS IO Filter",
                    DEFAULT_PRIORITY, // priority
                    QStringList{"las", "laz"},
                    "laz",
                    QStringList{"LAS file (*.las *.laz)"},
                    QStringList{"LAS file (*.las *.laz)"},
                    Import | Export})
{
}

CC_FILE_ERROR LASIOFilter::loadFile(const QString &fileName, ccHObject &container, LoadParameters &parameters)
{
    laszip_POINTER laszipReader{};
    laszip_header *laszipHeader{nullptr};
    laszip_BOOL isCompressed{false};
    laszip_CHAR *errorMsg{nullptr};

    if (laszip_create(&laszipReader))
    {
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    if (laszip_open_reader(laszipReader, qPrintable(fileName), &isCompressed))
    {
        laszip_get_error(laszipHeader, &errorMsg);
        ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    if (laszip_get_header_pointer(laszipReader, &laszipHeader))
    {
        laszip_get_error(laszipHeader, &errorMsg);
        ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    laszip_U64 pointCount;
    if (laszipHeader->version_minor == 4)
    {
        pointCount = laszipHeader->extended_number_of_point_records;
    }
    else
    {
        pointCount = laszipHeader->number_of_point_records;
    }

    auto pointCloud = std::make_unique<ccPointCloud>(QFileInfo(fileName).fileName());

    if (!pointCloud->reserve(pointCount))
    {
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_NOT_ENOUGH_MEMORY;
    }

    if (pointCount >= std::numeric_limits<unsigned int>::max())
    {
        // TODO handle when pointcount >= u32::max
        abort();
    }

    std::vector<LasScalarField> availableScalarFields =
        LasScalarFieldForPointFormat(laszipHeader->point_data_format);

    std::vector<LasExtraScalarField> availableEXtraScalarFields;
    auto *extraBytesVlr = std::find_if(laszipHeader->vlrs,
                                       laszipHeader->vlrs + laszipHeader->number_of_variable_length_records,
                                       isExtraBytesVlr);
    if (extraBytesVlr < laszipHeader->vlrs + laszipHeader->number_of_variable_length_records)
    {
        availableEXtraScalarFields = LasExtraScalarField::ParseExtraScalarFields(*extraBytesVlr);
    }

    LASOpenDialog dialog;
    dialog.setInfo(laszipHeader->version_minor, laszipHeader->point_data_format, pointCount);
    dialog.setAvailableScalarFields(availableScalarFields, availableEXtraScalarFields);
    dialog.exec();
    if (dialog.result() == QDialog::Rejected)
    {
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_CANCELED_BY_USER;
    }

    dialog.filterOutNotChecked(availableScalarFields, availableEXtraScalarFields);

    CCVector3d lasMins(laszipHeader->min_x, laszipHeader->min_y, laszipHeader->min_z);

    laszip_F64 laszipCoordinates[3];
    laszip_point *laszipPoint;
    CCVector3 currentPoint{};
    CCVector3d shift;
    bool preserveGlobalShift{true};
    pointCloud->enableScalarField();

    if (laszip_get_point_pointer(laszipReader, &laszipPoint))
    {
        laszip_get_error(laszipHeader, &errorMsg);
        ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
        return CC_FERR_NOT_ENOUGH_MEMORY;
    }

    ccLog::Print("Extra bytes: %d", availableEXtraScalarFields.size());
    LasScalarFieldLoader loader(availableScalarFields, availableEXtraScalarFields);

    QElapsedTimer timer;
    timer.start();

    ccProgressDialog progressDialog(true);
    progressDialog.setMethodTitle("Loading LAS points");
    progressDialog.setInfo("Loading points");
    CCCoreLib::NormalizedProgress normProgress(&progressDialog, pointCount);
    unsigned int numStepsForUpdate = 3 * pointCount / 100;
    unsigned int lastProgressUpdate = 0;
    progressDialog.show();

    CC_FILE_ERROR error;
    for (unsigned int i{0}; i < pointCount; ++i)
    {
        if (progressDialog.isCancelRequested())
        {
            error = CC_FERR_CANCELED_BY_USER;
            break;
        }

        if (laszip_read_point(laszipReader))
        {
            error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
            break;
        }

        if (laszip_get_coordinates(laszipReader, laszipCoordinates))
        {
            error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
            break;
        }

        if (i == 0)
        {
            CCVector3d firstPoint(laszipCoordinates[0], laszipCoordinates[1], laszipCoordinates[2]);
            shift = GetGlobalShift(parameters, preserveGlobalShift, lasMins, firstPoint);

            if (preserveGlobalShift)
            {
                pointCloud->setGlobalShift(shift);
            }

            if (shift.norm2() != 0.0)
            {
                ccLog::Warning("[LAS] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)",
                               shift.x,
                               shift.y,
                               shift.z);
            }
            pointCloud->setGlobalShift(shift);
        }

        currentPoint.x = static_cast<PointCoordinateType>(laszipCoordinates[0] + shift.x);
        currentPoint.y = static_cast<PointCoordinateType>(laszipCoordinates[1] + shift.y);
        currentPoint.z = static_cast<PointCoordinateType>(laszipCoordinates[2] + shift.z);

        pointCloud->addPoint(currentPoint);

        error = loader.handleScalarFields(*pointCloud, *laszipPoint);
        if (error != CC_FERR_NO_ERROR)
        {
            break;
        }

        error = loader.handleExtraScalarFields(*pointCloud, *laszipPoint);
        if (error != CC_FERR_NO_ERROR)
        {
            break;
        }

        if (HasRGB(laszipHeader->point_data_format))
        {
            error = loader.handleRGBValue(*pointCloud, *laszipPoint);
            if (error != CC_FERR_NO_ERROR)
            {
                break;
            }
        }

        if ((i - lastProgressUpdate) == numStepsForUpdate)
        {
            normProgress.steps(i - lastProgressUpdate);
            lastProgressUpdate += (i - lastProgressUpdate);
        }
    }

    // TODO print ignored scalar field

    if (pointCloud->hasColors())
    {
        pointCloud->showColors(true);
    }
    else if (pointCloud->getNumberOfScalarFields() > 0)
    {
        for (unsigned int i{0}; i < pointCloud->getNumberOfScalarFields(); ++i)
        {
            pointCloud->getScalarField(static_cast<int>(i))->computeMinAndMax();
        }
        pointCloud->setCurrentDisplayedScalarField(0);
    }

    LasSavedInfo info(*laszipHeader);
    info.extraScalarFields = availableEXtraScalarFields;
    pointCloud->setMetaData(LAS_METADATA_INFO_KEY, QVariant::fromValue(info));

    container.addChild(pointCloud.release());

    if (error == CC_FERR_THIRD_PARTY_LIB_FAILURE)
    {
        laszip_get_error(laszipHeader, &errorMsg);
        ccLog::Warning("[LAS] laszip error: '%s'", errorMsg);
        laszip_close_reader(laszipReader);
        laszip_clean(laszipReader);
        laszip_destroy(laszipReader);
    }

    ccLog::Print(QString("[LAS] File loaded in %1 seconds").arg(timer.elapsed()));
    return error;
}

bool LASIOFilter::canSave(CC_CLASS_ENUM type, bool &multiple, bool &exclusive) const
{
    multiple = false;
    exclusive = true;
    return type == CC_TYPES::POINT_CLOUD;
}

CC_FILE_ERROR LASIOFilter::saveToFile(ccHObject *entity,
                                      const QString &filename,
                                      const FileIOFilter::SaveParameters &parameters)
{
    if (!entity || filename.isEmpty())
    {
        return CC_FERR_BAD_ARGUMENT;
    }

    if (!entity->isA(CC_TYPES::POINT_CLOUD))
    {
        return CC_FERR_BAD_ENTITY_TYPE;
    }
    auto *pointCloud = static_cast<ccPointCloud *>(entity);

    if (!pointCloud->hasMetaData(LAS_METADATA_INFO_KEY))
    {
        ccLog::Warning("Cannot save cloud not loaded from las file");
        return CC_FERR_NOT_IMPLEMENTED;
    }

    auto savedInfo = qvariant_cast<LasSavedInfo>(pointCloud->getMetaData(LAS_METADATA_INFO_KEY));

    // optimal scale (for accuracy) --> 1e-9 because the maximum integer is roughly +/-2e+9
    CCVector3d bbMax;
    CCVector3d bbMin;
    if (!pointCloud->getGlobalBB(bbMin, bbMax))
    {
        return CC_FERR_NO_SAVE;
    }
    CCVector3d diag = bbMax - bbMin;
    CCVector3d optimalScale(1.0e-9 * std::max<double>(diag.x, CCCoreLib::ZERO_TOLERANCE_D),
                            1.0e-9 * std::max<double>(diag.y, CCCoreLib::ZERO_TOLERANCE_D),
                            1.0e-9 * std::max<double>(diag.z, CCCoreLib::ZERO_TOLERANCE_D));

    LASSaveDialog saveDialog(pointCloud);
    saveDialog.setOptimalScale(optimalScale);
    saveDialog.setSavedScale(CCVector3d(savedInfo.xScale, savedInfo.yScale, savedInfo.zScale));
    saveDialog.setVersionAndPointFormat(QString("1.%1").arg(QString::number(savedInfo.versionMinor)),
                                        savedInfo.pointFormat);

    saveDialog.exec();
    if (saveDialog.result() == QDialog::Rejected)
    {
        return CC_FERR_CANCELED_BY_USER;
    }

    laszip_header laszipHeader = InitLaszipHeader(saveDialog, savedInfo, *pointCloud);

    std::vector<LasScalarField> fieldsToSave = saveDialog.fieldsToSave();
    LasScalarFieldSaver fieldSaver(fieldsToSave, savedInfo.extraScalarFields);

    laszip_POINTER laszipWriter{nullptr};
    laszip_CHAR *errorMsg{nullptr};

    if (laszip_create(&laszipWriter))
    {
        ccLog::Warning("[LAS] laszip failed to create the writer");
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    if (laszip_set_header(laszipWriter, &laszipHeader))
    {
        laszip_get_error(laszipWriter, &errorMsg);
        ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
        laszip_destroy(laszipWriter);
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    if (laszip_open_writer(laszipWriter, qPrintable(filename), filename.endsWith("laz")))
    {
        laszip_get_error(laszipWriter, &errorMsg);
        ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
        laszip_destroy(laszipWriter);
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    laszip_point laszipPoint{};
    int totalExtraByteSize =
        laszipHeader.point_data_record_length - PointFormatSize(laszipHeader.point_data_format);
    if (totalExtraByteSize > 0)
    {
        laszipPoint.num_extra_bytes = totalExtraByteSize;
        laszipPoint.extra_bytes = new laszip_U8[totalExtraByteSize];
    }

    CC_FILE_ERROR error = CC_FERR_NO_ERROR;
    for (unsigned int i{0}; i < pointCloud->size(); ++i)
    {
        fieldSaver.handleScalarFields(i, laszipPoint);
        fieldSaver.handleExtraFields(i, laszipPoint);

        const CCVector3 *point = pointCloud->getPoint(i);
        if (pointCloud->isShifted())
        {
            laszipPoint.X = static_cast<laszip_I32>(point->x / laszipHeader.x_scale_factor);
            laszipPoint.Y = static_cast<laszip_I32>(point->y / laszipHeader.y_scale_factor);
            laszipPoint.Z = static_cast<laszip_I32>(point->z / laszipHeader.z_scale_factor);
        }
        else
        {
            CCVector3d globalPoint = pointCloud->toGlobal3d<PointCoordinateType>(*point);
            laszipPoint.X = static_cast<laszip_I32>((globalPoint.x - laszipHeader.x_offset) /
                                                    laszipHeader.x_scale_factor);
            laszipPoint.Y = static_cast<laszip_I32>((globalPoint.y - laszipHeader.y_offset) /
                                                    laszipHeader.y_scale_factor);
            laszipPoint.Z = static_cast<laszip_I32>((globalPoint.z - laszipHeader.z_offset) /
                                                    laszipHeader.z_scale_factor);
        }

        if (laszip_set_point(laszipWriter, &laszipPoint))
        {
            error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
            break;
        }

        if (laszip_write_point(laszipWriter))
        {
            error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
            break;
        }

        if (laszip_update_inventory(laszipWriter))
        {
            error = CC_FERR_THIRD_PARTY_LIB_FAILURE;
            break;
        }
    }

    if (error == CC_FERR_THIRD_PARTY_LIB_FAILURE)
    {
        laszip_get_error(laszipWriter, &errorMsg);
        ccLog::Warning("[LAS] laszip error :'%s'", errorMsg);
    }

    laszip_close_writer(laszipWriter);
    laszip_clean(laszipWriter);
    laszip_destroy(laszipWriter);
    return error;
}
