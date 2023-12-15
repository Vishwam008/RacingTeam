#include "status/process_frame.hpp"

adas_api::ProcessFrame::ProcessFrame( const uint32_t frameId ) : 
    frameId( frameId ), 
    _timestamp( std::chrono::steady_clock::now() )
{
}

bool adas_api::ProcessFrame::is_recent() const
{
    return std::chrono::steady_clock::now() - _timestamp < _timeout;    
}

void adas_api::ProcessFrame::process( const can_frame &frame )
{
    _timestamp = std::chrono::steady_clock::now();
    if( frame.can_id == this->frameId ) 
        _process( frame );
}

const adas_api::ProcessFrame::timestamp_t& adas_api::ProcessFrame::timestamp() const
{
    return _timestamp;
}
