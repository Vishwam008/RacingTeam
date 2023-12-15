#include "status/brake_frame.hpp"

const float adas_api::BrakeFrame::front_brake() const
{
    return candata_vcu2_ai_brake_hyd_press_f_pct_decode( data.hyd_press_f_pct );
}

const float adas_api::BrakeFrame::front_brake_requested() const
{
    return candata_vcu2_ai_brake_hyd_press_f_req_pct_decode( data.hyd_press_f_req_pct );
}

const float adas_api::BrakeFrame::rear_brake() const
{
    return candata_vcu2_ai_brake_hyd_press_r_pct_decode( data.hyd_press_r_pct );
}

const float adas_api::BrakeFrame::rear_brake_requested() const
{
    return candata_vcu2_ai_brake_hyd_press_r_req_pct_decode( data.hyd_press_r_req_pct );
}

const adas_api::BrakeFrame::BrakeState adas_api::BrakeFrame::brake_state() const
{
    return static_cast<BrakeState>( candata_vcu2_ai_brake_status_brk_decode( data.status_brk) );
}

const adas_api::BrakeFrame::EbsState adas_api::BrakeFrame::ebs_state() const
{
    return static_cast<EbsState> (candata_vcu2_ai_brake_status_ebs_decode(data.status_ebs));
}

void adas_api::BrakeFrame::_process( const can_frame& frame )
{
    if( candata_vcu2_ai_brake_unpack( &data, frame.data, frame.can_dlc ) )
    {
        throw std::runtime_error( "Failed to unpack brake frame" );
    }
}

void adas_api::BrakeFrame::_print( std::ostream& os ) const
{
    /**
     * !TODO: Write a meaningfull message!
     */
    os << "Brake Frame: " ;
}

adas_api::BrakeFrame::BrakeFrame() : 
    ProcessFrame( CANDATA_VCU2_AI_BRAKE_FRAME_ID )
{
}
