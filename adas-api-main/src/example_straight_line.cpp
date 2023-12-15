#include "simple_adas.hpp"

#include <iostream>
#include <thread>

int main( int argc, char* argv[] )
{
    adas_api::SimpleAdas adas { argc >= 2 ? argv[1] : "vcan0" };

    enum States
    {
        IDLE,
        SAVE_START,
        ACCELERATE,
        DECELERATE,
        DONE
    } state = IDLE;

    std::array<int,4> startingCount { 0, 0, 0, 0 };     // wheel counts at start of test

    while( true )
    {
        adas.read();                                    // get latest information from car

        if( !adas.go() ) state = IDLE;              // start over if mission is aborted

        switch( state )
        {
            case IDLE:
                std::cout << "IDLE" << std::endl;
                adas.set_rpm( 0 );                      // don't move
                adas.set_brakes( 0 );

                if( adas.go() )                     // wait for mission to start
                    state = SAVE_START;
                break;

            case SAVE_START:
                std::cout << "SAVE_START" << std::endl;
                for( int i=0; i<4; ++i )                // save starting wheel counts
                    startingCount[i] = adas.wheel_counts().count( i );

                state = ACCELERATE;
                break;

            case ACCELERATE:
                std::cout << "ACCELERATE" << std::endl;
                adas.set_rpm( 250 );                     // move
                adas.set_brakes( 0 );

                for( int i=0; i<4; ++i )                // check distance traveled
                {
                    std::cout << adas.wheel_counts().count( i ) - startingCount[i] << " ";
                    if( std::abs( adas.wheel_counts().count( i ) - startingCount[i] ) >= 2000 )
                        state = DECELERATE;             // start stopping when distance is reached
                }
                std::cout << std::endl;
                break;

            case DECELERATE:
                std::cout << "DECELERATE" << std::endl;
                adas.set_rpm( 0 );                      // stop
                adas.set_brakes( 100 );

                if( adas.wheel_speeds().stopped() )
                    state = DONE;
                break;

            case DONE:
                std::cout << "DONE" << std::endl;
                adas.finish();                          // car in safe state
                break;
        }

        adas.write();                                   // send commands to car

        std::this_thread::sleep_for( std::chrono::milliseconds(10) );
    }

	return 0;
}
