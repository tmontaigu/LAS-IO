//
// Created by Thomas on 18/11/2020.
//

#ifndef LASDETAILS_H
#define LASDETAILS_H

#include <CCTypes.h>
#include <vector>

class ccScalarField;

class QDataStream;

struct laszip_vlr;
typedef laszip_vlr laszip_vlr_struct;

#define LAS_VLR_HEADER_SIZE 54
#define SCAN_ANGLE_SCALE 0.06

namespace LasNames
{
constexpr const char *Intensity = "Intensity";
constexpr const char *ReturnNumber = "Return Number";
constexpr const char *NumberOfReturns = "Number Of Returns";
constexpr const char *ScanDirectionFlag = "Scan Direction Flag";
constexpr const char *EdgeOfFlightLine = "EdgeOfFlightLine";
constexpr const char *Classification = "Classificatopn";
constexpr const char *SyntheticFlag = "Synthetic Flag";
constexpr const char *KeypointFlag = "Keypoint Flag";
constexpr const char *WithheldFlag = "Withheld Flag";
constexpr const char *ScanAngleRank = "Scan Angle Rank";
constexpr const char *UserData = "User Data";
constexpr const char *PointSourceId = "Point Source ID";
constexpr const char *GpsTime = "Gps Time";

// 1.4 point format 6 stuff
constexpr const char *ScanAngle = "Scan Angle";
constexpr const char *ScannerChannel = "Scanner Channel";
constexpr const char *OverlapFlag = "Overlap Flag";
constexpr const char *NearInfrared = "Near Infrared";
} // namespace LasNames

struct LasScalarField
{
    enum Id
    {
        Intensity = 0,
        ReturnNumber = 1,
        NumberOfReturns = 2,
        ScanDirectionFlag,
        EdgeOfFlightLine,
        Classification,
        SyntheticFlag,
        KeypointFlag,
        WithheldFlag,
        ScanAngleRank,
        UserData,
        PointSourceId,
        GpsTime,
        // Extended (LAS 1.4)
        ExtendedScanAngle,
        ExtendedScannerChannel,
        OverlapFlag,
        ExtendedClassification,
        ExtendedReturnNumber,
        ExtendedNumberOfReturns,
        NearInfrared
    };

    explicit constexpr LasScalarField(LasScalarField::Id id, ccScalarField *sf = nullptr);

    LasScalarField(const char *name, LasScalarField::Id id, ccScalarField *sf = nullptr)
        : name(name), id(id), sf(sf)
    {
    }

    constexpr static const char *NameFromId(LasScalarField::Id id);
    static LasScalarField::Id IdFromName(const char *name, unsigned int targetPointFormat);

    const char *name;
    Id id;
    ccScalarField *sf{nullptr};
};

uint16_t PointFormatSize(unsigned int pointFormat);

uint16_t HeaderSize(unsigned int versionMinor);

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

inline bool HasNearInfrared(unsigned int pointFormatId)
{
    return pointFormatId == 8 || pointFormatId == 10;
}

extern const char *AvailableVersions[3];

const std::vector<unsigned int> *PointFormatsAvailableForVersion(const char *version);

std::vector<LasScalarField> LasScalarFieldForPointFormat(unsigned int pointFormatId);

bool isLaszipVlr(const laszip_vlr_struct &);

bool isExtraBytesVlr(const laszip_vlr_struct &);

struct LasExtraScalarField
{
    enum DataType
    {
        Undocumented,
        u8,
        u16,
        u32,
        u64,
        i8,
        i16,
        i32,
        i64,
        f32,
        f64,
        u8_2,
        u16_2,
        u32_2,
        u64_2,
        i8_2,
        i16_2,
        i32_2,
        i64_2,
        f32_2,
        f64_2,
        u8_3,
        u16_3,
        u32_3,
        u64_3,
        i8_3,
        i16_3,
        i32_3,
        i64_3,
        f32_3,
        f64_3,
        Invalid
    };

    enum Kind {
        Signed,
        Unsigned,
        Floating
    };

  public:
    explicit LasExtraScalarField(QDataStream &dataStream);
    static std::vector<LasExtraScalarField> ParseExtraScalarFields(const laszip_vlr_struct &extraBytesVlr);

    unsigned int byteSize() const;
    unsigned int numElements() const;
    Kind kind() const;

    // Options
    bool noDataIsRelevant() const;
    bool minIsRelevant() const;
    bool maxIsRelevant() const;
    bool scaleIsRelevant() const;
    bool offsetIsRelevant() const;

    static DataType DataTypeFromValue(uint8_t value);

    // These info are from the vlr itself
    DataType type;
    uint8_t options;
    char name[32] = "";
    char description[32] = "";
    uint8_t noData[3][8] = {0};
    uint8_t mins[3][8] = {0};
    uint8_t maxs[3][8] = {0};
    double scales[3] = {0.0};
    double offsets[3] = {0.0};

    // These are added by us
    unsigned int byteOffset = {0};
    ccScalarField *scalarFields[3] = {nullptr};
};

class IExtraByteParser
{
  public:
    virtual ScalarType parseFrom(uint8_t *data) = 0;
    virtual ~IExtraByteParser() = default;
};

class SignedIntegeParser : public IExtraByteParser
{
  public:
    ScalarType parseFrom(uint8_t *data) override;

  private:
    const LasExtraScalarField *eb{nullptr};
};




#endif // LASDETAILS_H
