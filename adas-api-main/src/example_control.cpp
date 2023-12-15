#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <string>

#include <can_wrap.hpp>
using can::operator<<;

#include <control_frames.hpp>
#include <status_frames.hpp>

int main( int argc, char* argv[] )
{
    const std::string canChannel = "vcan0";

    adas_api::AxleTorqueFrame frontAxle;
    adas_api::AxleTorqueFrame rearAxle( false );
    adas_api::BrakeFrame;
    adas_api::StatusFrame status;
    adas_api::SteeringFrame steering;
    adas_api::WheelCountsFrame wheelCounts;
    adas_api::WheelSpeedsFrame wheelSpeeds;

    adas_api::AxleControlFrame frontControl( frontAxle, wheelSpeeds, status );
    adas_api::AxleControlFrame rearControl( rearAxle, wheelSpeeds, status, false );
    adas_api::StatusControlFrame statusControl( frontAxle, rearAxle, status );
    adas_api::SteeringControlFrame steeringControl( steering, status );

    std::array< adas_api::ProcessFrame* const, 6 > frames { &frontAxle, &rearAxle, &status, &steering, &wheelCounts, &wheelSpeeds };
    std::array< adas_api::ControlFrame* const, 4 > controls { &frontControl, &rearControl, &statusControl, &steeringControl };

    try
    {
        const int canSocket = can::connect( canChannel );

        for( int c=0; c<100000; ++c )
        {
            const can_frame frame = can::read( canSocket );

            // read all in
            for( auto& f : frames )
            {
                if( f->frameId == frame.can_id )
                {
                    f->process( frame );
                    std::cout << *f << std::endl;
                }
            }

            // write all out
            for( auto& f : controls )
            {
                can::write( canSocket, f->process() );
            }
        }

        can::close( canSocket );
    }
    catch( const std::exception& e )
    {
        std::cerr << e.what() << std::endl;
    }

	return 0;
}
