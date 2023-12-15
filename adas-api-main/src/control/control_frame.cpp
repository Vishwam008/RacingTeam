#include "control/control_frame.hpp"

adas_api::ControlFrame::ControlFrame( const uint32_t frameId ) : 
    frameId( frameId )
{
}

const can_frame adas_api::ControlFrame::process()
{
    return _process();
}

