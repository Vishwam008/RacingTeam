#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include "simple_adas.hpp"

int main( int argc, char* argv[] )
{
    adas_api::SimpleAdas adas { argc >= 2 ? argv[1] : "can0" }; // was "vcan0"

    enum States
    {
        IDLE,
        LEFT,
        RIGHT,
        CENTER,
        INCREMENT,
        DONE
    } state = IDLE;

    using clock = std::chrono::steady_clock;
    clock::time_point timer;

    const auto timeout = std::chrono::seconds( 3 );

    int count = 0;

    while( true )
    {
        States prev = state;

        if( adas.go() )
        {
            switch( state )
            {
                case IDLE:
                    state = LEFT;
                    break;

                case LEFT:
                    if( clock::now() - timer >= timeout )
                        state = RIGHT;
                    adas.set_angle( 20.f );
                    break;

                case RIGHT:
                    if( clock::now() - timer >= timeout )
                        state = CENTER;
                    adas.set_angle( -20.f );
                    break;

                case CENTER:
                    if( clock::now() - timer >= timeout )
                        state = INCREMENT;
                    adas.set_angle( 0.f );
                    break;

                case INCREMENT:
                    count += 1;
                    if( count < 20 )
                        state = IDLE;
                    else
                        state = DONE;
                    break;

                case DONE:
                    adas.finish();
                    break;
            }
        }
        else
            state = IDLE;

        if( state != prev )
            timer = clock::now();

        adas.read();
        adas.write();

        std::this_thread::sleep_for( std::chrono::milliseconds(10) );
    }

	return 0;
}
