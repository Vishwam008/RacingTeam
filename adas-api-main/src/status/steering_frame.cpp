#include "status/steering_frame.hpp"
#include "calibration.hpp"

void adas_api::SteeringFrame::set_calibration( const std::vector<float>& calibration )
{
    this->_calibration = calibration;
}

float adas_api::SteeringFrame::angle() const
{
    const float raw = candata_vcu2_ai_steer_angle_decode( data.angle );

    if( this->_calibration.empty() )
        return raw;;

    return calibration( this->_calibration, raw );
}

float adas_api::SteeringFrame::requested() const
{
    return candata_vcu2_ai_steer_angle_request_decode( data.angle_request );
}

float adas_api::SteeringFrame::max() const
{
    return candata_vcu2_ai_steer_angle_max_decode( data.angle_max );
}

void adas_api::SteeringFrame::_process( const can_frame& frame )
{
    if( candata_vcu2_ai_steer_unpack( &data, frame.data, frame.can_dlc ) )
    {
        throw std::runtime_error( "Failed to unpack steering frame" );
    }
}

void adas_api::SteeringFrame::_print( std::ostream& os ) const
{
    os << "Angle " << angle() << " of " << requested() << "/" << max();
}

adas_api::SteeringFrame::SteeringFrame() : 
            ProcessFrame( CANDATA_VCU2_AI_STEER_FRAME_ID )
{}