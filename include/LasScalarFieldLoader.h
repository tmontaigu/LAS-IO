#ifndef LASSCALARFIELDLOADER_H
#define LASSCALARFIELDLOADER_H

#include "LasDetails.h"

#include <FileIOFilter.h>
#include <QFileInfo>

#include <vector>

#include <ccPointCloud.h>
#include <laszip/laszip_api.h>


class LasScalarFieldLoader
{
  public:
    LasScalarFieldLoader(std::vector<LasScalarField> standardScalarFields,
                         std::vector<LasExtraScalarField> extraScalarFields,
                         ccPointCloud &pointCloud);

    CC_FILE_ERROR handleScalarFields(ccPointCloud &pointCloud, const laszip_point &currentPoint);

    CC_FILE_ERROR handleRGBValue(ccPointCloud &pointCloud, const laszip_point &currentPoint);

    CC_FILE_ERROR handleExtraScalarFields(ccPointCloud &pointCloud, const laszip_point &currentPoint);

  private:
    template <typename T>
    CC_FILE_ERROR handleScalarField(LasScalarField &sfInfo, ccPointCloud &pointCloud, T currentValue);

    CC_FILE_ERROR handleGpsTime(LasScalarField &sfInfo, ccPointCloud &pointCloud, double currentValue);

    bool createScalarFieldsForExtraBytes(ccPointCloud &pointCloud);

    template <typename T> static ScalarType ParseValueOfType(uint8_t *source);

    template <typename T, typename V> static V ParseValueOfTypeAs(const uint8_t *source);

    void parseRawValues(const LasExtraScalarField &extraField, uint8_t *dataStart);

    template <typename T> void handleOptionsFor(const LasExtraScalarField &extraField, T values[3]);

  private:
    unsigned char colorCompShift{0};
    std::vector<LasScalarField> m_standardFields{};
    std::vector<LasExtraScalarField> m_extraScalarFields{};

    union
    {
        uint64_t unsignedValues[3];
        int64_t signedValues[3];
        double floatingValues[3];
    } rawValues{};
};

#endif // LASSCALARFIELDLOADER_H
