#include "status/wheel_counts_frame.hpp"


const adas_api::WheelCountsFrame::counts_t adas_api::WheelCountsFrame::count( size_t index ) const
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

const adas_api::WheelCountsFrame::counts_t adas_api::WheelCountsFrame::front_left() const
{
    return candata_vcu2_ai_wheel_counts_fl_pulse_count_decode( data.fl_pulse_count );
}

const adas_api::WheelCountsFrame::counts_t adas_api::WheelCountsFrame::front_right() const
{
    return candata_vcu2_ai_wheel_counts_fr_pulse_count_decode( data.fr_pulse_count );
}

const adas_api::WheelCountsFrame::counts_t adas_api::WheelCountsFrame::rear_left() const
{
    return candata_vcu2_ai_wheel_counts_rl_pulse_count_decode( data.rl_pulse_count );
}

const adas_api::WheelCountsFrame::counts_t adas_api::WheelCountsFrame::rear_right() const
{
    return candata_vcu2_ai_wheel_counts_rr_pulse_count_decode( data.rr_pulse_count );
}

void adas_api::WheelCountsFrame::_process( const can_frame& frame )
{
    if( candata_vcu2_ai_wheel_counts_unpack( &data, frame.data, frame.can_dlc ) )
    {
        throw std::runtime_error( "Failed to unpack wheel counts frame" );
    }
}

void adas_api::WheelCountsFrame::_print( std::ostream& os ) const
{
    os << "Wheel counts: " << count(0);
    for( size_t i=1; i<4; ++i )
        os << ", " << count(i);
}

adas_api::WheelCountsFrame::WheelCountsFrame() : 
    ProcessFrame( CANDATA_VCU2_AI_WHEEL_COUNTS_FRAME_ID )
{
}

adas_api::WheelCountsFrame::~WheelCountsFrame()
{
}
