#include "status/axle_torque_frame.hpp"

float adas_api::AxleTorqueFrame::get_current() const
{
    if( is_front() )
        return candata_vcu2_ai_drive_f_front_axle_trq_max_decode( 
            std::get<candata_vcu2_ai_drive_f_t>(data).front_axle_trq );

    return candata_vcu2_ai_drive_r_rear_axle_trq_max_decode( 
        std::get<candata_vcu2_ai_drive_r_t>(data).rear_axle_trq );
}

float adas_api::AxleTorqueFrame::get_max() const
{
    if( is_front() )
        return candata_vcu2_ai_drive_f_front_axle_trq_max_decode( 
            std::get<candata_vcu2_ai_drive_f_t>(data).front_axle_trq_max );

    return candata_vcu2_ai_drive_r_rear_axle_trq_max_decode( 
        std::get<candata_vcu2_ai_drive_r_t>(data).rear_axle_trq_max );
}

float adas_api::AxleTorqueFrame::get_requested() const
{
    if( is_front() )
        return candata_vcu2_ai_drive_f_front_axle_trq_max_decode( 
            std::get<candata_vcu2_ai_drive_f_t>(data).front_axle_trq_request );

    return candata_vcu2_ai_drive_r_rear_axle_trq_max_decode( 
        std::get<candata_vcu2_ai_drive_r_t>(data).rear_axle_trq_request );
}

void adas_api::AxleTorqueFrame::_process( const can_frame& frame )
{
    if( is_front() )
    {
        if( candata_vcu2_ai_drive_f_unpack( &std::get<candata_vcu2_ai_drive_f_t>(data), frame.data, frame.can_dlc ) )
        {   
            throw std::runtime_error( "Failed to unpack front torque information frame" );
        }
    }
    else if( candata_vcu2_ai_drive_r_unpack( &std::get<candata_vcu2_ai_drive_r_t>(data), frame.data, frame.can_dlc ) )
    {
        throw std::runtime_error( "Failed to unpack rear torque information frame" );
    }
}

bool adas_api::AxleTorqueFrame::is_front() const
{
    return std::holds_alternative<candata_vcu2_ai_drive_f_t>( data );
}

bool adas_api::AxleTorqueFrame::stopped() const
{
    return std::abs( 0.f - this->get_current() ) < std::numeric_limits<float>::epsilon();
}

void adas_api::AxleTorqueFrame::_print( std::ostream& os ) const
{
    os << ( is_front() ? "Front" : "Rear" ) << " torque: "
        << "current: " << get_current()
        << ", max: " << get_max()
        << ", requested: " << get_requested();
}

adas_api::AxleTorqueFrame::AxleTorqueFrame( const bool front ) : 
    ProcessFrame( front ? CANDATA_VCU2_AI_DRIVE_F_FRAME_ID : CANDATA_VCU2_AI_DRIVE_R_FRAME_ID )
{
    if( front )
        data.emplace<candata_vcu2_ai_drive_f_t>();
    else
        data.emplace<candata_vcu2_ai_drive_r_t>();
}

