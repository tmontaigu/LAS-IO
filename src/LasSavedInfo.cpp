#include "LasSavedInfo.h"

#include <algorithm>
#include <cstring>

LasSavedInfo::LasSavedInfo(const laszip_header &header)
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

    numVlrs =
        std::count_if(header.vlrs, header.vlrs + header.number_of_variable_length_records, vlrShouldBeCopied);

    if (numVlrs > 0)
    {
        vlrs = new laszip_vlr_struct[numVlrs];
        std::copy_if(
            header.vlrs, header.vlrs + header.number_of_variable_length_records, vlrs, vlrShouldBeCopied);
    }
}

LasSavedInfo::LasSavedInfo(const LasSavedInfo &rhs)
    : fileSourceId(rhs.fileSourceId), guidData1(rhs.guidData1), guidData2(rhs.guidData2),
      guidData3(rhs.guidData3), versionMinor(rhs.versionMinor), pointFormat(rhs.pointFormat),
      xScale(rhs.xScale), yScale(rhs.yScale), zScale(rhs.zScale), numVlrs(rhs.numVlrs),
      extraScalarFields(rhs.extraScalarFields)
{

    strncpy(guidData4, rhs.guidData4, 8);
    strncpy(systemIdentifier, rhs.systemIdentifier, 32);
    if (numVlrs > 0)
    {
        vlrs = new laszip_vlr_struct[numVlrs];
        std::copy(rhs.vlrs, rhs.vlrs + rhs.numVlrs, vlrs);
    }
}
LasSavedInfo::~LasSavedInfo() noexcept
{
    delete[] vlrs;
}
