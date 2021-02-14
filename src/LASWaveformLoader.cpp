
#include "LASWaveformLoader.h"

bool parseWavepacketDescriptorVlr(const laszip_vlr_struct &vlr, WaveformDescriptor &descriptor)
{
    if (vlr.record_length_after_header < 26)
    {
        return false;
    }

    auto data =
        QByteArray::fromRawData(reinterpret_cast<const char *>(vlr.data), vlr.record_length_after_header);
    QDataStream stream(data);

    uint8_t compressionType;
    stream >> descriptor.bitsPerSample >> compressionType >> descriptor.numberOfSamples >>
        descriptor.samplingRate_ps >> descriptor.digitizerGain >> descriptor.digitizerOffset;

    if (descriptor.digitizerGain == 0.0)
    {
        // shouldn't be 0 by default!
        descriptor.digitizerGain = 1.0;
    }

    return true;
}

void parseWaveformDescriptorVlrs(const laszip_vlr_struct *vlrs,
                                 unsigned int numVlrs,
                                 ccPointCloud &pointCloud)
{

    for (size_t i{0}; i < numVlrs; ++i)
    {
        const laszip_vlr_struct *vlr = &vlrs[i];
        if (strcmp(vlr->user_id, "LASF_Spec") == 0 && 100 <= vlr->record_id && vlr->record_id <= 354)
        {
            WaveformDescriptor descriptor;
            if (!parseWavepacketDescriptorVlr(*vlr, descriptor))
            {
                ccLog::Warning("[LAS] Invalid Descriptor VLR");
            }
            else
            {
                ccLog::Print("Id: %d, index: %d", vlr->record_id, vlr->record_id -100);
                pointCloud.fwfDescriptors().insert(vlr->record_id - 100, descriptor);
            }
        }
    }
}

LasWaveformLoader::LasWaveformLoader(const laszip_header_struct &laszipHeader,
                                     const QString &lasFilename,
                                     ccPointCloud &pointCloud)
    : isPointFormatExtended(laszipHeader.point_data_format >= 6)
{
    parseWaveformDescriptorVlrs(
        laszipHeader.vlrs, laszipHeader.number_of_variable_length_records, pointCloud);
    ccLog::Print("[LAS] %d Waveform Packet Descriptor VLRs found", pointCloud.fwfDescriptors().size());

    QFile fwfDataSource;
    if (laszipHeader.start_of_waveform_data_packet_record != 0)
    {
        ccLog::Print("[LAS] Waveform data is located within the las file");
        fwfDataSource.setFileName(lasFilename);
        if (!fwfDataSource.open(QFile::ReadOnly))
        {
            ccLog::Warning(
                QString("[LAS] Failed to re open the las file: %1").arg(fwfDataSource.errorString()));
            return;
        }

        if (!fwfDataSource.seek(laszipHeader.start_of_waveform_data_packet_record))
        {
            ccLog::Warning(QString("[LAS] Failed to find the associated waveform data packets header"));
            return;
        }

        QByteArray evlrHeader = fwfDataSource.read(60);
        if (evlrHeader.size() < 60)
        {
            ccLog::Warning(QString("[LAS] Failed to read the associated waveform data packets"));
            return;
        }

        unsigned short recordID = *reinterpret_cast<const unsigned short *>(evlrHeader.constData() + 18);
        if (recordID != 65535)
        {
            ccLog::Warning("[LAS] Invalid waveform EVLR");
            return;
        }
        fwfDataCount = *reinterpret_cast<const uint64_t *>(evlrHeader.constData() +
                                                           20); // see LAS 1.4 EVLR header specifications
        if (fwfDataCount == 0)
        {
            ccLog::Warning(QString("[LAS] Invalid waveform data packet size (0). We'll load all the "
                                   "remaining part of the file!"));
            fwfDataCount = fwfDataSource.size() - fwfDataSource.pos();
        }
    }
    else if (laszipHeader.global_encoding & 4)
    {
        QFileInfo info(lasFilename);
        QString wdpFilename = QString("%1/%2.wdp").arg(info.path()).arg(info.baseName());
        fwfDataSource.setFileName(wdpFilename);
        if (!fwfDataSource.open(QFile::ReadOnly))
        {
            ccLog::Warning(QString("[LAS] Failed to read the associated waveform data packets file "
                                   "(looking for '%1'): %2")
                               .arg(wdpFilename)
                               .arg(fwfDataSource.errorString()));
            return;
        }
        fwfDataCount = fwfDataSource.size();

        if (fwfDataCount > 60)
        {
            QByteArray evlrHeader = fwfDataSource.read(60);
            const char *userID = reinterpret_cast<const char *>(evlrHeader.constData() +
                                                                2); // see LAS 1.4 EVLR header specifications
            if (strncmp(userID, "LASF_Spec", 9) == 0)
            {
                // this is a valid EVLR header, we can skip it
                fwfDataCount -= 60;
            }
            else
            {
                // this doesn't look like a valid EVLR
                fwfDataSource.seek(0);
            }
        }
        ccLog::Print(
            QString("[LAS] Waveform Data Packets are in an external file located at %1").arg(wdpFilename));
    }

    if (fwfDataSource.isOpen() && fwfDataCount != 0)
    {
        ccPointCloud::FWFDataContainer *container{nullptr};
        try
        {
            container = new ccPointCloud::FWFDataContainer;
            container->resize(fwfDataCount);
            pointCloud.waveforms().resize(pointCloud.capacity());
        }
        catch (const std::bad_alloc &)
        {
            ccLog::Warning(QString("[LAS] Not enough memory to import the waveform data"));
            delete container;
            return;
        }

        fwfDataSource.read((char *)container->data(), fwfDataCount);
        fwfDataSource.close();

        pointCloud.fwfData() = ccPointCloud::SharedFWFDataContainer(container);
    }
}

void LasWaveformLoader::loadWaveform(ccPointCloud &pointCloud, const laszip_point &currentPoint) const
{
    if (fwfDataCount == 0)
    {
        return;
    }

    auto data = QByteArray::fromRawData(reinterpret_cast<const char *>(currentPoint.wave_packet), 29);
    QDataStream stream(data);
    uint8_t descriptorIndex;
    uint64_t byteOffset;
    uint32_t byteCount;
    float returnPointLocation;
    float x_t, y_t, z_t;
    stream >> descriptorIndex >> byteOffset >> byteCount >> returnPointLocation >> x_t >> y_t >> z_t;
    ccLog::Print("Index: %d", (int) descriptorIndex);

    if (descriptorIndex == 0)
    {
        // 0 means no waveform info for this point
        return;
    }
    descriptorIndex--;

    ccPointCloud::FWFDescriptorSet &descriptors = pointCloud.fwfDescriptors();
    if (!descriptors.contains(descriptorIndex))
    {
        ccLog::Warning("[LAS] No valid descriptor vlr for index %d", descriptorIndex);
        return;
    }

    ccWaveform &w = pointCloud.waveforms()[pointCloud.size() - 1];

    w.setDescriptorID(descriptorIndex);
    w.setDataDescription(byteOffset - fwfDataOffset, byteCount);
    w.setEchoTime_ps(returnPointLocation);
    w.setBeamDir(CCVector3f(x_t, y_t, z_t));

    if (isPointFormatExtended)
    {
        w.setReturnIndex(currentPoint.extended_return_number);
    }
    else
    {
        w.setReturnIndex(currentPoint.return_number);
    }
}
