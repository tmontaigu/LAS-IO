#ifndef CLOUDCOMPAREPROJECTS_LASWAVEFORMSAVER_H
#define CLOUDCOMPAREPROJECTS_LASWAVEFORMSAVER_H

#include <QByteArray>
#include <QDataStream>

#include <laszip/laszip_api.h>

class ccPointCloud;

struct LasWaveformSaver
{

    LasWaveformSaver(const ccPointCloud &pointCloud) noexcept;

    void handlePoint(size_t index, laszip_point &point);

  private:
    QByteArray m_array;
    const ccPointCloud &m_pointCloud;
};

#endif // CLOUDCOMPAREPROJECTS_LASWAVEFORMSAVER_H
