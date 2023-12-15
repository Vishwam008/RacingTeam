#include "control/brake_control_frame.hpp"
#include <candata.h>
#include <stdexcept>


void adas_api::BrakeControlFrame::set_front( const float percentage )
{
    this->_front = std::max( 0.f, std::min( 100.f, percentage ) );
}

void adas_api::BrakeControlFrame::set_rear( const float percentage )
{
    this->_rear = std::max( 0.f, std::min( 100.f, percentage ) );
}

void adas_api::BrakeControlFrame::set_brakes( const float percentage )
{
    this->set_front( percentage );
    this->set_rear( percentage );
}

float adas_api::BrakeControlFrame::get_front() const
{
    return this->_front;
}

float adas_api::BrakeControlFrame::get_rear() const
{
    return this->_rear;
}

bool adas_api::BrakeControlFrame::on() const
{
    return this->_front > 0.f || this->_rear > 0.f;
}

can_frame adas_api::BrakeControlFrame::_process() const
{
    float front = this->_front;
    float rear = this->_rear;

    can_frame frame
        {
            .can_id = this->frameId,
            .can_dlc = CANDATA_AI2_VCU_BRAKE_LENGTH
        };

    const candata_ai2_vcu_brake_t data
        {
            .hyd_press_f_req_pct = candata_ai2_vcu_brake_hyd_press_f_req_pct_encode( front ),
            .hyd_press_r_req_pct = candata_ai2_vcu_brake_hyd_press_r_req_pct_encode( rear )
        };

    if( !candata_ai2_vcu_brake_hyd_press_f_req_pct_is_in_range( data.hyd_press_f_req_pct ) )
        throw std::runtime_error( "Front brake out of range" );

    if( !candata_ai2_vcu_brake_hyd_press_r_req_pct_is_in_range( data.hyd_press_r_req_pct ) )
        throw std::runtime_error( "Rear brake out of range" );

    candata_ai2_vcu_brake_pack( frame.data, &data, frame.can_dlc );

    return frame;
}

void adas_api::BrakeControlFrame::_print( std::ostream& os ) const
{
}

adas_api::BrakeControlFrame::BrakeControlFrame() : 
    ControlFrame( CANDATA_AI2_VCU_BRAKE_FRAME_ID )
{}

