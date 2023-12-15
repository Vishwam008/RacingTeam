#include "control/steering_control_frame.hpp"

#include <candata.h>
#include <calibration.hpp>

#include <stdexcept>

void adas_api::SteeringControlFrame::set_calibration( const std::vector<float>& calibration )
{
    this->_calibration = calibration;
}

/**
 * @brief 
 * 
 * Going to accept setting of any angle and then limit it before it is sent
 * as the steering angle limit may not/isn't always known when the value is set.
 * 
 * @param angle 
 */
void adas_api::SteeringControlFrame::set_angle( const float angle )
{
    _angle = angle;
}

can_frame adas_api::SteeringControlFrame::_process() const
{
    float angle = 0.f;

    /* check that we have up to date information otherwise keep as safe values */
    if( this->_steeringFrame.is_recent() && this->_statusFrame.is_recent() )
    {
        switch( this->_statusFrame.as_state() )
        {
            default:
                [[fallthrough]];
            case StatusFrame::AsState::OFF:
                [[fallthrough]];
            case StatusFrame::AsState::READY:
                [[fallthrough]];
            case StatusFrame::AsState::FINISHED:
                break;

            case StatusFrame::AsState::EMERGENCY:
                [[fallthrough]];
            case StatusFrame::AsState::DRIVING:
                const float mx = std::abs( this->_steeringFrame.max() );
                // limit angle to max steering angle
                // use same direction as steering frame

                if( this->_calibration.empty() )
                    angle = this->_angle;
                else
                    angle = calibration( this->_calibration, this->_angle );

                angle = std::min( mx, std::max( -mx, angle ) );
                break;
        }
    }

    can_frame frame
        {
            .can_id = this->frameId,
            .can_dlc = CANDATA_AI2_VCU_STEER_LENGTH
        };

    const candata_ai2_vcu_steer_t data
        {
            .steer_request = candata_ai2_vcu_steer_steer_request_encode( angle )
        };
 
    if( !candata_ai2_vcu_steer_steer_request_is_in_range( data.steer_request ) )
    {
        throw std::runtime_error( "Steering angle out of range" );
    }

    candata_ai2_vcu_steer_pack( frame.data, &data, frame.can_dlc );
 
    return frame;
}

void adas_api::SteeringControlFrame::_print( std::ostream& os ) const
{
}

adas_api::SteeringControlFrame::SteeringControlFrame( const SteeringFrame &steeringFrame, 
                                                    const StatusFrame &statusFrame ) : 
    ControlFrame( CANDATA_AI2_VCU_STEER_FRAME_ID ),
    _steeringFrame( steeringFrame ),
    _statusFrame( statusFrame )
{}
