#ifndef LASSCALARFIELDSAVER_H
#define LASSCALARFIELDSAVER_H

#include "LASDetails.h"

#include <vector>

class ccPointCloud;
struct laszip_point;

class LasScalarFieldSaver
{
  public:
    LasScalarFieldSaver(std::vector<LasScalarField> standardFields,
                        std::vector<LasExtraScalarField> extraFields);

    void handleScalarFields(size_t i, laszip_point &point);

    void handleExtraFields(size_t i, laszip_point &point);

  private:
    template <typename T> static void WriteScalarValueAs(ScalarType value, uint8_t *dest)
    {
        if (value > std::numeric_limits<T>::max())
        {
            *reinterpret_cast<T *>(dest) = std::numeric_limits<T>::max();
        }
        else if (value < std::numeric_limits<T>::min())
        {
            *reinterpret_cast<T *>(dest) = std::numeric_limits<T>::min();
        }
        else
        {
            *reinterpret_cast<T *>(dest) = static_cast<T>(value);
        }
    }

  private:
    std::vector<LasScalarField> m_standardFields;
    std::vector<LasExtraScalarField> m_extraFields;
};

#endif // LASSCALARFIELDSAVER_H
