#ifndef LASSAVEDINFO_H
#define LASSAVEDINFO_H

#include "LasDetails.h"

#include <laszip/laszip_api.h>

#include <vector>

/// Holds Meta-Information about the original file that we want to save
/// to restore them when writing
struct LasSavedInfo
{
    LasSavedInfo() = default;

    explicit LasSavedInfo(const laszip_header &header);

    LasSavedInfo(const LasSavedInfo &rhs);

    virtual ~LasSavedInfo() noexcept;

    laszip_U16 fileSourceId{};
    laszip_U32 guidData1{};
    laszip_U16 guidData2{};
    laszip_U16 guidData3{};
    laszip_CHAR guidData4[8]{};
    laszip_U8 versionMinor{};
    laszip_U8 pointFormat{};
    laszip_CHAR systemIdentifier[32]{};
    double xScale{0.0};
    double yScale{0.0};
    double zScale{0.0};

    laszip_U32 numVlrs{0};
    laszip_vlr_struct *vlrs{nullptr};
    std::vector<LasExtraScalarField> extraScalarFields{};
};

#endif // LASSAVEDINFO_H
