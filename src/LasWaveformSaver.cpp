#include "LasWaveformSaver.h"

#include <LasDetails.h>
#include <ccPointCloud.h>

LasWaveformSaver::LasWaveformSaver(const ccPointCloud &pointCloud) noexcept
    : m_array(29, '\0'), m_pointCloud(pointCloud)
{
}

void LasWaveformSaver::handlePoint(size_t index, laszip_point &point)
{
    Q_ASSERT(index < m_pointCloud.size());
    const ccWaveform w = m_pointCloud.waveforms()[index];

    {
        QDataStream stream(&m_array, QIODevice::WriteOnly);
        stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);
        stream << w.descriptorID();
        stream << static_cast<quint64>(w.dataOffset() + EvlrHeader::SIZE);
        stream << w.byteCount();
        stream << w.echoTime_ps();
        stream << w.beamDir().x << w.beamDir().y << w.beamDir().z;
    }

    memcpy(point.wave_packet, m_array.constData(), 29);
}