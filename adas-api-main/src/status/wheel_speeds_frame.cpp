#include "status/wheel_speeds_frame.hpp"


const adas_api::WheelSpeedsFrame::rpm_t adas_api::WheelSpeedsFrame::rpm( size_t index ) const
{
    switch( index )
    {
        case 0:
            return front_left();
        case 1:
            return front_right();
        case 2:
            return rear_left();
        case 3:
            return rear_right();
    }

    return 0;
}

const adas_api::WheelSpeedsFrame::rpm_t adas_api::WheelSpeedsFrame::front_left() const
{
    return candata_vcu2_ai_speeds_fl_wheel_speed_decode( data.fl_wheel_speed );
}

const adas_api::WheelSpeedsFrame::rpm_t adas_api::WheelSpeedsFrame::front_right() const
{
    return candata_vcu2_ai_speeds_fr_wheel_speed_decode( data.fr_wheel_speed );
}

const adas_api::WheelSpeedsFrame::rpm_t adas_api::WheelSpeedsFrame::rear_left() const
{
    return candata_vcu2_ai_speeds_rl_wheel_speed_decode( data.rl_wheel_speed );
}

const adas_api::WheelSpeedsFrame::rpm_t adas_api::WheelSpeedsFrame::rear_right() const
{
    return candata_vcu2_ai_speeds_rr_wheel_speed_decode( data.rr_wheel_speed );
}

const adas_api::WheelSpeedsFrame::rpm_t adas_api::WheelSpeedsFrame::fastest() const
{
    return std::max( std::max( front_left(), front_right() ), 
                     std::max( rear_left(), rear_right() ) );
}

const bool adas_api::WheelSpeedsFrame::stopped() const
{
	return this->front_left() == 0 &&
		    this->front_right() == 0 &&
		    this->rear_left() == 0 &&
		    this->rear_right() == 0;
}

void adas_api::WheelSpeedsFrame::_process( const can_frame& frame )
{
    if( candata_vcu2_ai_speeds_unpack( &data, frame.data, frame.can_dlc ) )
    {
        throw std::runtime_error( "Failed to unpack wheel counts frame" );
    }
}

void adas_api::WheelSpeedsFrame::_print( std::ostream& os ) const
{
    os << "Wheel counts: " << rpm(0);
    for( size_t i=1; i<4; ++i )
        os << ", " << rpm(i);
}

adas_api::WheelSpeedsFrame::WheelSpeedsFrame() : 
    ProcessFrame( CANDATA_VCU2_AI_SPEEDS_FRAME_ID )
{
}
