#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <string>

#include <can_wrap.hpp>
using can::operator<<;

#include <status_frames.hpp>

int main( int argc, char* argv[] )
{
    const std::string canChannel = "vcan0";

    adas_api::AxleTorqueFrame frontAxle;
    adas_api::AxleTorqueFrame rearAxle( false );
    adas_api::StatusFrame status;
    adas_api::SteeringFrame steering;
    adas_api::WheelCountsFrame wheelCounts;
    adas_api::WheelSpeedsFrame wheelSpeeds;

    std::array< adas_api::ProcessFrame* const, 6 > frames { &frontAxle, &rearAxle, &status, &steering, &wheelCounts, &wheelSpeeds };

    try
    {
        const int canSocket = can::connect( canChannel );

        for( int c=0; c<100000; ++c )
        {
            const can_frame frame = can::read( canSocket );

            for( auto& f : frames )
            {
                if( f->frameId == frame.can_id )
                {
                    f->process( frame );
                    std::cout << *f << std::endl;
                }
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
