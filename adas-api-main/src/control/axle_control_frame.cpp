#include "control/axle_control_frame.hpp"
#include <cmath>

void adas_api::AxleControlFrame::set_speed( const float speed )
{
    _speed = speed;
}

void adas_api::AxleControlFrame::set_torque( const float torque )
{
    _torque = torque;
}

void adas_api::AxleControlFrame::set_plaid( const bool plaid )
{
    _plaidMode = plaid;
}

can_frame adas_api::AxleControlFrame::_process() const
{
    float speed = 0.f;
    float torque = 0.f;

    /* check that we have up to date information otherwise set safe values */
    if( this->_brakeControl.on() == false && 
        this->_statusFrame.is_recent() && 
        this->_statusFrame.as_state() == adas_api::StatusFrame::AsState::DRIVING &&   // in driving mode
        /*this->_brakeFrame.is_recent() && 
        this->_brakeFrame.on() == false &&*/                                  // brakes are off
        this->_torqueFrame.is_recent() && 
        this->_speedsFrame.is_recent() )
    {
        speed = std::max( 0.f, std::min( this->_speed * adas_api::AxleControlFrame::motor_ratio, 4000.f ) );

        const float maxAllowedTorque = std::min( this->_torqueFrame.get_max(),
                                                   torque_rpm_limit( this->_speedsFrame.fastest() ) );

        torque = std::max( 0.f, std::min( maxAllowedTorque, this->_torque ) );
    }

    can_frame frame 
        {
            .can_id = this->frameId
        };

    if( is_front() )
    {
        frame.can_dlc = CANDATA_AI2_VCU_DRIVE_F_LENGTH;
        const candata_ai2_vcu_drive_f_t data
            {
                .front_axle_trq_request = candata_ai2_vcu_drive_f_front_axle_trq_request_encode( torque ),
                .front_motor_speed_max = candata_ai2_vcu_drive_f_front_motor_speed_max_encode( speed )
            };

        if( !candata_ai2_vcu_drive_f_front_axle_trq_request_is_in_range( data.front_axle_trq_request ) )
            throw std::runtime_error( "front torque out of range" );

        if( !candata_ai2_vcu_drive_f_front_motor_speed_max_is_in_range( data.front_motor_speed_max ) )
            throw std::runtime_error( "Rear motor speed out of range" );
        
        candata_ai2_vcu_drive_f_pack( frame.data, &data, frame.can_dlc );
    }
    else
    {
        frame.can_dlc = CANDATA_AI2_VCU_DRIVE_R_LENGTH;
        const candata_ai2_vcu_drive_r_t data
            {
                .rear_axle_trq_request = candata_ai2_vcu_drive_r_rear_axle_trq_request_encode( torque ),
                .rear_motor_speed_max = candata_ai2_vcu_drive_r_rear_motor_speed_max_encode( speed )
            };

        if( !candata_ai2_vcu_drive_r_rear_axle_trq_request_is_in_range( data.rear_axle_trq_request ) )
            throw std::runtime_error( "Rear torque out of range" );

        if( !candata_ai2_vcu_drive_r_rear_motor_speed_max_is_in_range( data.rear_motor_speed_max ) )
            throw std::runtime_error( "Rear motor speed out of range" );

        candata_ai2_vcu_drive_r_pack( frame.data, &data, frame.can_dlc );
    }

    return frame;
}

bool adas_api::AxleControlFrame::is_front() const
{
    return _front;
}

void adas_api::AxleControlFrame::_print( std::ostream& os ) const
{
}

float adas_api::AxleControlFrame::_plaid_mode( const float rpm )
{
    // cavern opt F
    /*if( rpm > 500 ) return 150;
    else if( rpm > 470 ) return -0.33 * rpm + 316.6;
    else if( rpm > 430 ) return -0.25 * rpm + 277.5;
    else if( rpm > 390 ) return -0.55 * rpm + 406.5;
    else return 192.f;*/

    // cavern opt H
    /*if( rpm > 510 ) return 150;
    else if( rpm > 398 ) return 75096 * pow( rpm, -0.998 );
    return 195.f;*/

    return std::min( 200.f, std::max( 50.f, 309.25f - ( 0.2875f * rpm ) ) );
}// so the second number of 306.25 | safe 286.25 is definetly safe | can change it to 309.25

float adas_api::AxleControlFrame::torque_rpm_limit( const float rpm ) const
{
    if( _plaidMode ) return _plaid_mode( rpm );
    
    if( rpm > 700 )
        return 50.f;
    else if( rpm > 600 )
        return 85.f;
    else if( rpm > 500 )
        return 100.f;
    else if( rpm > 400 )
        return 120.f;
    else if( rpm > 300 )
        return 150.f;

    return 200.f;
}

adas_api::AxleControlFrame::AxleControlFrame( const adas_api::AxleTorqueFrame &torqueFrame,
                                    const adas_api::BrakeFrame &brakeFrame,
                                    const adas_api::WheelSpeedsFrame &speedsFrame,
                                    const adas_api::StatusFrame &statusFrame,
                                    const adas_api::BrakeControlFrame &brakeControl,
                                    const bool front ) : 
    ControlFrame( front ? CANDATA_AI2_VCU_DRIVE_F_FRAME_ID : CANDATA_AI2_VCU_DRIVE_R_FRAME_ID ),
    _torqueFrame( torqueFrame ),
    _brakeFrame( brakeFrame ),
    _speedsFrame( speedsFrame ),
    _statusFrame( statusFrame ),
    _brakeControl( brakeControl ),
    _front( front )
{
    if( front != torqueFrame.is_front() )
        throw std::logic_error( "Front axle being paired with rear axle." );
}
