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

#include "LASIOFilter.h"

#include <QtCore/QElapsedTimer>
#include <ccPointCloud.h>
#include <laszip/laszip_api.h>

#define RETURN_IF_ERROR(errorValue)                                                                          \
    if (errorValue != CC_FERR_NO_ERROR)                                                                      \
    {                                                                                                        \
        return errorValue;                                                                                   \
    }

inline bool HasGpsTime(unsigned int pointFormatId)
{
    return pointFormatId == 1 || pointFormatId == 3 || pointFormatId == 5 || pointFormatId >= 6;
}

inline bool HasRGB(unsigned int pointFormatId)
{
    return pointFormatId == 2 || pointFormatId == 3 || pointFormatId == 4 || pointFormatId == 5 ||
           pointFormatId >= 7;
}

inline bool HasWaveform(unsigned int pointFormatId)
{
    return pointFormatId == 4 || pointFormatId == 5 || pointFormatId >= 8;
}

struct ExtraBytes
{
    unsigned char dataType{0};
    unsigned char options{0};
    char name[32] = "";
    uint8_t noData[3][8] = {0};
    uint8_t mins[3][8] = {0};
    uint8_t maxs[3][8] = {0};
    double scales[3] = {0.0};
    double offsets[3] = {0.0};
    char description[32] = "";


    QString n;
    bool isFromArray;


    explicit ExtraBytes(QDataStream &data)
    {
        data.skipRawData(2);
        data >> dataType >> options;
        data.readRawData(name, 32);
        data.readRawData(reinterpret_cast<char *>(noData), 3 * 8);
        data.readRawData(reinterpret_cast<char *>(mins), 3 * 8);
        data.readRawData(reinterpret_cast<char *>(maxs), 3 * 8);
        data >> scales[0] >> scales[1] >> scales[2];
        data >> offsets[0] >> offsets[1] >> offsets[2];
        data.readRawData(reinterpret_cast<char *>(description), 3 * 8);
    }
};

static std::vector<ExtraBytes> FindExtraBytesVLR(const laszip_vlr *vlrs, int num_vlrs)
{
    std::vector<ExtraBytes> extraFields;

    if (!vlrs || num_vlrs == 0)
    {
        return extraFields;
    }

    for (int i{0}; i < num_vlrs; ++i)
    {
        if (strcmp(vlrs[i].user_id, "LASF_Spec") == 0 && vlrs[i].record_id == 4)
        {
            int numExtraFields = vlrs[i].record_length_after_header / 192;
            extraFields.reserve(numExtraFields);

            QByteArray byteArray(reinterpret_cast<char *>(vlrs[i].data), vlrs[i].record_length_after_header);
            QDataStream dataStream(byteArray);

            for (int j{0}; j < numExtraFields; ++j)
            {
                extraFields.emplace_back(dataStream);
            }
        }
    }
    return extraFields;
}

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

template <typename T>
static CC_FILE_ERROR
HandleScalarFieldValue(const char *sfName, ccPointCloud &pointCloud, T currentValue, T firstValue)
{
    Q_ASSERT(pointCloud.size() > 0);

    int idx = pointCloud.getScalarFieldIndexByName(sfName);
    if (idx == -1 && currentValue != firstValue)
    {
        idx = pointCloud.addScalarField(sfName);
        CCCoreLib::ScalarField *sf = pointCloud.getScalarField(idx);
        if (!sf->reserveSafe(pointCloud.capacity()))
        {
            return CC_FERR_NOT_ENOUGH_MEMORY;
        }
        for (unsigned int j{0}; j < pointCloud.size() - 1; ++j)
        {
            sf->addElement(static_cast<ScalarType>(firstValue));
        }
    }

    if (idx >= 0)
    {
        pointCloud.getScalarField(idx)->addElement(static_cast<ScalarType>(currentValue));
    }
    return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR
HandleRGBValue(ccPointCloud &pointCloud, const laszip_point &currentPoint, const laszip_point &firstPoint)
{
    if (!pointCloud.hasColors() &&
        memcmp(currentPoint.rgb, firstPoint.rgb, sizeof(((laszip_point *)0)->rgb)) != 0)
    {
        ccLog::Print("Adding colors");
        if (!pointCloud.reserveTheRGBTable())
        {
            return CC_FERR_NOT_ENOUGH_MEMORY;
        }
        for (unsigned int j{0}; j < pointCloud.size() - 1; ++j)
        {

            auto red = static_cast<ColorCompType>(firstPoint.rgb[0] >> 8);
            auto green = static_cast<ColorCompType>(firstPoint.rgb[1] >> 8);
            auto blue = static_cast<ColorCompType>(firstPoint.rgb[2] >> 8);
            pointCloud.addColor(ccColor::Rgb(red, green, blue));
        }
    }

    if (pointCloud.hasColors())
    {
        auto red = static_cast<ColorCompType>(currentPoint.rgb[0] >> 8);
        auto green = static_cast<ColorCompType>(currentPoint.rgb[1] >> 8);
        auto blue = static_cast<ColorCompType>(currentPoint.rgb[2] >> 8);
        pointCloud.addColor(ccColor::Rgb(red, green, blue));
    }
    return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR
HandleWaveform(ccPointCloud &pointCloud, const laszip_point &currentPoint, const laszip_point &firstPoint)
{
    if (!pointCloud.hasFWF() &&
        memcmp(currentPoint.wave_packet, firstPoint.wave_packet, sizeof(((laszip_point *)0)->wave_packet)) !=
            0)
    {
        if (!pointCloud.reserveTheFWFTable())
        {
            return CC_FERR_NOT_ENOUGH_MEMORY;
        }
        for (unsigned int j{0}; j < pointCloud.size() - 1; ++j)
        {
            ccWaveform &w = pointCloud.waveforms()[j];
            //            w.setDescriptorID(packetIndex);
            //            w.setDataDescription(point.wavepacket.getOffset() - fwfDataOffset,
            //            point.wavepacket.getSize()); w.setBeamDir(
            //                CCVector3f(point.wavepacket.getXt(), point.wavepacket.getYt(),
            //                point.wavepacket.getZt()));
            //            w.setEchoTime_ps(point.wavepacket.getLocation());
            //            w.setReturnIndex(point.return_number);
        }
    }

    if (pointCloud.hasFWF()) {}
    return CC_FERR_NO_ERROR;
}

/// For extra bytes we always load them
static CC_FILE_ERROR HandleExtraScalaFields(ccPointCloud &pointCloud,
                                            const std::vector<ExtraBytes> &extraFields,
                                            const laszip_point &currentPoint)
{

    return CC_FERR_NO_ERROR;
}

static CC_FILE_ERROR HandleLegacyPointScalarFields(ccPointCloud &pointCloud,
                                                   const laszip_point &currentPoint,
                                                   const laszip_point &firstPoint)
{

    CC_FILE_ERROR error =
        HandleScalarFieldValue("Intensity", pointCloud, currentPoint.intensity, firstPoint.intensity);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Return Number", pointCloud, currentPoint.return_number, firstPoint.return_number);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Number Of Returns", pointCloud, currentPoint.number_of_returns, firstPoint.number_of_returns);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Scan Direction Flag", pointCloud, currentPoint.scan_direction_flag, firstPoint.scan_direction_flag);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Edge Of Flight Line", pointCloud, currentPoint.edge_of_flight_line, firstPoint.edge_of_flight_line);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Classification", pointCloud, currentPoint.classification, firstPoint.classification);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Synthetic Flag", pointCloud, currentPoint.synthetic_flag, firstPoint.synthetic_flag);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Keypoint Flag", pointCloud, currentPoint.keypoint_flag, firstPoint.keypoint_flag);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Withhled Flag", pointCloud, currentPoint.withheld_flag, firstPoint.withheld_flag);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Scan Angle Rank", pointCloud, currentPoint.scan_angle_rank, firstPoint.scan_angle_rank);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue("User Data", pointCloud, currentPoint.user_data, firstPoint.user_data);
    RETURN_IF_ERROR(error);

    error = HandleScalarFieldValue(
        "Point Source ID", pointCloud, currentPoint.point_source_ID, firstPoint.point_source_ID);
    RETURN_IF_ERROR(error);
    return CC_FERR_NO_ERROR;
}

LASIOFilter::LASIOFilter()
    : FileIOFilter({"LAS IO Filter",
                    DEFAULT_PRIORITY, // priority
                    QStringList{"las", "laz"},
                    "laz",
                    QStringList{"LAS file (*.las *.laz)"},
                    QStringList(),
                    Import})
{
}

CC_FILE_ERROR LASIOFilter::loadFile(const QString &fileName, ccHObject &container, LoadParameters &parameters)
{
    QElapsedTimer timer;
    timer.start();

    laszip_POINTER laszipReader{};
    laszip_header *laszipHeader{nullptr};
    laszip_BOOL isCompressed{false};

    // TODO handle cleanup of laszip things on each error

    if (laszip_create(&laszipReader))
    {
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    if (laszip_open_reader(laszipReader, qPrintable(fileName), &isCompressed))
    {
        // TODO get error from laszip
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    if (laszip_get_header_pointer(laszipReader, &laszipHeader))
    {
        // TODO get error from laszip
        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    }

    // TODO check after reading that the return count is the same as the header
    // I think this just ocunts points read/ written but does not look in the header
    //    if (laszip_get_point_count(laszipReader, &pointCount))
    //    {
    //        // TODO get error from laszip
    //        return CC_FERR_THIRD_PARTY_LIB_FAILURE;
    //    }

    laszip_U64 pointCount;
    if (laszipHeader->version_minor == 4)
    {
        pointCount = laszipHeader->extended_number_of_point_records;
    }
    else
    {
        pointCount = laszipHeader->number_of_point_records;
    }
    // TODO handle when pointcount >= u32::max

    ccLog::Print("[LAS-IO] This file has %ld points", pointCount);

    auto pointCloud = std::make_unique<ccPointCloud>("las file");

    if (!pointCloud->reserve(pointCount))
    {
        return CC_FERR_NOT_ENOUGH_MEMORY;
    }

    if (pointCount >= std::numeric_limits<unsigned int>::max())
    {
        abort();
    }

    CCVector3d lasMins(laszipHeader->min_x, laszipHeader->min_y, laszipHeader->min_z);

    laszip_F64 laszipCoordinates[3];
    laszip_point *laszipPoint;
    laszip_point laszipFirstPoint;
    CCVector3 currentPoint{};
    CCVector3d shift;
    bool preserveGlobalShift{true};
    pointCloud->enableScalarField();

    if (laszip_get_point_pointer(laszipReader, &laszipPoint))
    {
        return CC_FERR_NOT_ENOUGH_MEMORY;
    }

    std::vector<ExtraBytes> extraFields;
    {
        if (laszip_read_point(laszipReader))
        {
            return CC_FERR_THIRD_PARTY_LIB_FAILURE;
        }

        if (laszip_get_coordinates(laszipReader, laszipCoordinates))
        {
            return CC_FERR_THIRD_PARTY_LIB_FAILURE;
        }

        laszipFirstPoint = *laszipPoint;

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

        currentPoint.x = static_cast<PointCoordinateType>(laszipCoordinates[0] - laszipHeader->min_x);
        currentPoint.y = static_cast<PointCoordinateType>(laszipCoordinates[1] - laszipHeader->min_y);
        currentPoint.z = static_cast<PointCoordinateType>(laszipCoordinates[2] - laszipHeader->min_z);

        pointCloud->addPoint(currentPoint);
        if (laszipFirstPoint.num_extra_bytes > 0)
        {
            extraFields =
                FindExtraBytesVLR(laszipHeader->vlrs, laszipHeader->number_of_variable_length_records);
            if (extraFields.empty())
            {
                ccLog::Print(QString("Points have %1 extra bytes but no extra bytes vlr was found")
                                 .arg(QString::number(laszipFirstPoint.num_extra_bytes)));
                // TODO somehow find a way to keep them to restore when writing ?
            }
        }
    }

    CC_FILE_ERROR error = CC_FERR_NO_ERROR;
    for (unsigned int i{1}; i < pointCount; ++i)
    {

        if (laszip_read_point(laszipReader))
        {
            return CC_FERR_THIRD_PARTY_LIB_FAILURE;
        }

        if (laszip_get_coordinates(laszipReader, laszipCoordinates))
        {
            return CC_FERR_THIRD_PARTY_LIB_FAILURE;
        }
        // TODO use shift;
        currentPoint.x = static_cast<PointCoordinateType>(laszipCoordinates[0] - laszipHeader->min_x);
        currentPoint.y = static_cast<PointCoordinateType>(laszipCoordinates[1] - laszipHeader->min_y);
        currentPoint.z = static_cast<PointCoordinateType>(laszipCoordinates[2] - laszipHeader->min_z);

        pointCloud->addPoint(currentPoint);

        if (laszipHeader->point_data_format < 6)
        {
            error = HandleLegacyPointScalarFields(*pointCloud, *laszipPoint, laszipFirstPoint);
            RETURN_IF_ERROR(error);
        }
        else
        {
            return CC_FERR_NOT_IMPLEMENTED;
        }

        if (HasRGB(laszipHeader->point_data_format))
        {
            RETURN_IF_ERROR(HandleRGBValue(*pointCloud, *laszipPoint, laszipFirstPoint));
        }

        if (HasGpsTime(laszipHeader->point_data_format))
        {
            RETURN_IF_ERROR(HandleScalarFieldValue(
                "GpsTime", *pointCloud, laszipPoint->gps_time, laszipFirstPoint.gps_time));
        }

        if (HasWaveform(laszipHeader->point_data_format)) {}
    }

    // TODO print ignored scalar field bc they were all equal
    // TODO set a visible scalarfield

    for (int i{0}; i < pointCloud->getNumberOfScalarFields(); ++i)
    {
        pointCloud->getScalarField(i)->computeMinAndMax();
    }

    container.addChild(pointCloud.release());
    ccLog::Print(QString("[LAS] File loaded in %1 seconds").arg(timer.elapsed() / 1'000));
    return CC_FERR_NO_ERROR;
}

bool LASIOFilter::canSave(CC_CLASS_ENUM type, bool &multiple, bool &exclusive) const
{
    Q_UNUSED(type);
    Q_UNUSED(multiple);
    Q_UNUSED(exclusive);

    // ... can we save this?
    return false;
}
