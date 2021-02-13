#ifndef LASWAVEFORMLOADER_H
#define LASWAVEFORMLOADER_H

#include <QString>
struct laszip_vlr;
typedef laszip_vlr laszip_vlr_struct;
class ccPointCloud;
struct laszip_point;
#include <QFileInfo>
#include <ccPointCloud.h>
#include <laszip/laszip_api.h>
bool parseWavepacketDescriptorVlr(const laszip_vlr_struct &vlr, WaveformDescriptor &descriptor);

struct LasWaveformLoader
{

    LasWaveformLoader(const laszip_header_struct &laszipHeader,
                      const QString &lasFilename,
                      ccPointCloud &pointCloud);

    void loadWaveform(ccPointCloud &pointCloud, const laszip_point &currentPoint) const;

    unsigned int fwfDataCount{0};
    unsigned int fwfDataOffset{0};
    bool isPointFormatExtended{false};
};

#endif // LASWAVEFORMLOADER_H
