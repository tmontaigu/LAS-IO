#ifndef LASSCALARFIELDSAVER_H
#define LASSCALARFIELDSAVER_H

#include "LASDetails.h"

#include <vector>

class ccPointCloud;
struct laszip_point;

class LasScalarFieldSaver
{
  public:
    explicit LasScalarFieldSaver(std::vector<LasScalarField> standardFields);

    void handleScalarFields(size_t i, laszip_point &point);

  private:
    std::vector<LasScalarField> m_standardFields;
};

#endif // LASSCALARFIELDSAVER_H
