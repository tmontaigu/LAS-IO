#include "LasScalarFieldSaver.h"

#include <laszip/laszip_api.h>

#include <ccScalarField.h>

LasScalarFieldSaver::LasScalarFieldSaver(std::vector<LasScalarField> standardFields)
    : m_standardFields(std::move(standardFields))
{
}

void LasScalarFieldSaver::handleScalarFields(size_t i, laszip_point &point)
{
    for (const LasScalarField &field : m_standardFields)
    {
        switch (field.id)
        {
        case LasScalarField::Intensity:
            if (field.sf)
            {
                point.intensity = static_cast<laszip_U16>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::ReturnNumber:
            if (field.sf)
            {
                point.return_number = static_cast<laszip_U8>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::NumberOfReturns:
            if (field.sf)
            {
                point.number_of_returns = static_cast<laszip_U8>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::ScanDirectionFlag:
            if (field.sf)
            {
                point.scan_direction_flag = (static_cast<laszip_U8>((*(field.sf))[i]) > 0) ? 1 : 0;
            }
            break;
        case LasScalarField::EdgeOfFlightLine:
            if (field.sf)
            {
                point.edge_of_flight_line = (static_cast<laszip_U8>((*(field.sf))[i]) > 0) ? 1 : 0;
            }
            break;
        case LasScalarField::Classification:
            if (field.sf)
            {
                point.classification = static_cast<laszip_U8>((*(field.sf))[i]) & 0b00011111;
            }
        case LasScalarField::SyntheticFlag:
            if (field.sf)
            {
                point.synthetic_flag = (static_cast<laszip_U8>((*(field.sf))[i]) > 0) ? 1 : 0;
            }
            break;
        case LasScalarField::KeypointFlag:
            if (field.sf)
            {
                point.keypoint_flag = (static_cast<laszip_U8>((*(field.sf))[i]) > 0) ? 1 : 0;
            }
            break;
        case LasScalarField::WithheldFlag:
            if (field.sf)
            {
                point.withheld_flag = (static_cast<laszip_U8>((*(field.sf))[i]) > 0) ? 1 : 0;
            }
            break;
        case LasScalarField::ScanAngleRank:
            if (field.sf)
            {
                point.extended_scan_angle = static_cast<laszip_I16>((*(field.sf))[i] / SCAN_ANGLE_SCALE);
            }
        case LasScalarField::UserData:
            if (field.sf)
            {
                point.user_data = static_cast<laszip_U8>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::PointSourceId:
            if (field.sf)
            {
                point.point_source_ID = static_cast<laszip_U16>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::GpsTime:
            if (field.sf)
            {
                point.gps_time = static_cast<laszip_F64>((*(field.sf))[i]) + field.sf->getGlobalShift();
            }
            break;
        case LasScalarField::ExtendedScanAngle:
            if (field.sf)
            {
                point.extended_scan_angle = static_cast<laszip_I16>((*(field.sf))[i] / SCAN_ANGLE_SCALE);
            }
            break;
        case LasScalarField::ExtendedScannerChannel:
            if (field.sf)
            {
                point.extended_scanner_channel = static_cast<laszip_U8>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::OverlapFlag:
            if (field.sf)
            {
                point.extended_classification_flags |=
                    (static_cast<laszip_U8>((*(field.sf))[i]) > 0) ? (1u << 5) : 0;
            }
            break;
        case LasScalarField::ExtendedClassification:
            if (field.sf)
            {
                point.extended_classification = static_cast<laszip_U8>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::ExtendedReturnNumber:
            if (field.sf)
            {
                point.extended_return_number = static_cast<laszip_U16>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::ExtendedNumberOfReturns:
            if (field.sf)
            {
                point.extended_number_of_returns = static_cast<laszip_U16>((*(field.sf))[i]);
            }
            break;
        case LasScalarField::NearInfrared:
            if (field.sf)
            {
                point.rgb[3] = static_cast<laszip_U16>((*(field.sf))[i]);
            }
            break;
        }
    }
}
