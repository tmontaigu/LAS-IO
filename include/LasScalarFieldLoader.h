#ifndef LASSCALARFIELDLOADER_H
#define LASSCALARFIELDLOADER_H

#include "LASDetails.h"

#include <FileIOFilter.h>
#include <QFileInfo>

#include <vector>

class ccPointCloud;
struct laszip_point;

struct laszip_vlr;
typedef laszip_vlr laszip_vlr_struct;

#include <ccPointCloud.h>
#include <laszip/laszip_api.h>

struct LasWaveformLoader {

    LasWaveformLoader(const laszip_header_struct& laszipHeader, const QString &lasFilename, ccPointCloud& pointCloud) {
        if (laszipHeader.start_of_waveform_data_packet_record != 0)
        {
            ccLog::Print("Waveform data is located in the file");
            fwfDataSource.setFileName(lasFilename);
            if (!fwfDataSource.open(QFile::ReadOnly))
            {
                ccLog::Warning(QString("Failed to read the associated waveform data packets"));
            }

            if (!fwfDataSource.seek(laszipHeader.start_of_waveform_data_packet_record))
            {
                ccLog::Warning(QString("Failed to find the associated waveform data packets header"));
            }

            QByteArray evlrHeader = fwfDataSource.read(60);
            if (evlrHeader.size() < 60)
            {
                ccLog::Warning(QString("Failed to read the associated waveform data packets"));
            }

            unsigned short recordID = *reinterpret_cast<const unsigned short *>(evlrHeader.constData() + 18);
            assert(recordID == 65535);
            fwfDataCount = *reinterpret_cast<const uint64_t *>(evlrHeader.constData() +
                                                               20); // see LAS 1.4 EVLR header specifications
            if (fwfDataCount == 0)
            {
                ccLog::Warning(QString(
                    "Invalid waveform data packet size (0). We'll load all the remaining part of the file!"));
                fwfDataCount = fwfDataSource.size() - fwfDataSource.pos();
            }
        }
        else if (laszipHeader.global_encoding & 4)
        {
            ccLog::Print("Waveform should be located in another file");
            QString wdpFilename = lasFilename;
            wdpFilename.replace(QFileInfo(lasFilename).suffix(), "wdp");
            fwfDataSource.setFileName(wdpFilename);
            if (!fwfDataSource.open(QFile::ReadOnly))
            {
                ccLog::Warning(
                    QString("Failed to read the associated waveform data packets file (looking for '%1')")
                        .arg(wdpFilename));
            }

            fwfDataCount = fwfDataSource.size();

            if (fwfDataCount > 60)
            {
                QByteArray evlrHeader = fwfDataSource.read(60);
                const char *userID = reinterpret_cast<const char *>(
                    evlrHeader.constData() + 2); // see LAS 1.4 EVLR header specifications
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
        }

        if (fwfDataSource.isOpen() && fwfDataCount != 0)
        {
            auto *container = new ccPointCloud::FWFDataContainer;
            try
            {
                container->resize(fwfDataCount);
            }
            catch (const std::bad_alloc &)
            {
                ccLog::Warning(QString("Not enough memory to import the waveform data"));
                // cloud->waveforms().clear();
                delete container;
            }

            fwfDataSource.read((char *)container->data(), fwfDataCount);
            fwfDataSource.close();

            pointCloud.fwfData() = ccPointCloud::SharedFWFDataContainer(container);
        }

        if (fwfDataCount)
        {
            try
            {
                pointCloud.waveforms().resize(pointCloud.capacity());
            }
            catch (const std::bad_alloc &)
            {
                ccLog::Warning(QString("Not enough memory to import the waveform data"));
            }
        }
    }



    QFile fwfDataSource;
    unsigned int fwfDataCount{0};
    unsigned int fwfDataOffset{0};
    unsigned int numWavepacketDecr{0};
    laszip_vlr_struct *wavepacketsDescrs{nullptr};


    void readWaveform(ccPointCloud& pointCloud, const laszip_point& currentPoint)
    {
        if (fwfDataCount == 0) {
            return;
        }

        ccPointCloud::FWFDescriptorSet &descriptors = pointCloud.fwfDescriptors();

        unsigned int descrIndex = currentPoint.wave_packet[0];
        if (!descriptors.contains(descrIndex) && descrIndex < numWavepacketDecr)
        {
			const laszip_vlr &descriptor = wavepacketsDescrs[descrIndex];
            WaveformDescriptor wfd;
            wfd.bitsPerSample = *descriptor.data;
            wfd.numberOfSamples = *reinterpret_cast<uint32_t*>((descriptor.data + 2));
            wfd.samplingRate_ps = *reinterpret_cast<uint32_t *>((descriptor.data + 6));
            wfd.digitizerGain = *reinterpret_cast<double *>((descriptor.data + 10));
            if (wfd.digitizerGain == 0)
            {
                // shouldn't be 0 by default!
                wfd.digitizerGain = 1.0;
            }
            wfd.digitizerOffset = *reinterpret_cast<double *>((descriptor.data + 18));
            descriptors.insert(descrIndex, wfd);

            ccWaveform &w = pointCloud.waveforms()[pointCloud.size() - 1];
            w.setDescriptorID(descrIndex);
            w.setDataDescription(*reinterpret_cast<const uint64_t*>(currentPoint.wave_packet + 1) - fwfDataOffset,
                                 *reinterpret_cast<const uint32_t *>(currentPoint.wave_packet + 9));
            w.setEchoTime_ps(*reinterpret_cast<const float *>(currentPoint.wave_packet + 13));
            w.setBeamDir(
                CCVector3f(*reinterpret_cast<const float *>(currentPoint.wave_packet + 17),
                                    *reinterpret_cast<const float *>(currentPoint.wave_packet + 21),
                                    *reinterpret_cast<const float *>(currentPoint.wave_packet + 25)));

            w.setReturnIndex(currentPoint.return_number); //TODO for extended
        }
    }
};

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
