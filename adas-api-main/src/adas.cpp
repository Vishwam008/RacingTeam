#include "adas.hpp"

#include <fcntl.h>

namespace adas_api
{
    void Adas::read( const int n )
    {
        try
        {
            for( int i=0; i<n; ++i )
            {
                const can_frame rawFrame = can::read( this->_canSocket );

                if( auto statusFrame = this->statusFrames.find( rawFrame.can_id ); 
                    statusFrame != this->statusFrames.end() )
                {
                    statusFrame->second->process( rawFrame );
                }
            }
        }
        catch( const can::error& )
        {
            // no more frames to read
        }
    }

    void Adas::write()
    {
        for( auto controlFrame : this->controlFrames )
        {
            can::write( this->_canSocket, controlFrame->process() );
        }
    }

    Adas::Adas( const std::string& device ) : _canSocket( can::connect( device ) )
    {
        // set socket to non-blocking
        const int existingFlags = fcntl( this->_canSocket, F_GETFL, 0 );
        fcntl( this->_canSocket, F_SETFL, existingFlags | O_NONBLOCK );
    }

    Adas::~Adas()
    {
        try
        {
            can::close( this->_canSocket );
        }
        catch( const can::error& )
        {}
    }
};