#ifndef LASWAVEFORMLOADER_H
#define LASWAVEFORMLOADER_H

#include <QFileInfo>
#include <QString>

#include <ccPointCloud.h>

#include <laszip/laszip_api.h>

struct LasWaveformLoader
{

    LasWaveformLoader(const laszip_header_struct &laszipHeader,
                      const QString &lasFilename,
                      ccPointCloud &pointCloud);

    void loadWaveform(ccPointCloud &pointCloud, const laszip_point &currentPoint) const;

    unsigned int fwfDataCount{0};
    unsigned int fwfDataOffset{0};
    bool isPointFormatExtended{false};
    ccPointCloud::FWFDescriptorSet descriptors;
};

#endif // LASWAVEFORMLOADER_H
