#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <fstream>

#include "simple_adas.hpp"

std::vector<float> read_calibration_file( const std::string filename )
{
    std::ifstream file( filename );
    if( !file.good() )
        throw std::runtime_error( "Failed to open file: " + filename );

    std::vector<float> data;

    std::copy( std::istream_iterator<float>(file), 
               std::istream_iterator<float>(), 
               std::back_inserter(data) );

    return data;
}

int main( int argc, char* argv[] )
{
    adas_api::SimpleAdas adas { argc >= 2 ? argv[1] : "can0" };

    auto steeringRequestCalib = read_calibration_file( "docs/steering_request.txt" );
    auto steeringFeedbackCalib = read_calibration_file( "docs/steering_feedback.txt" );

    adas.set_steering_request_calibration( steeringRequestCalib );
    adas.set_steering_feedback_calibration( steeringFeedbackCalib );

    enum States
    {
        IDLE,
        LEFT,
        RIGHT,
        CENTER,
        DONE
    } state = IDLE;

    using clock = std::chrono::steady_clock;
    clock::time_point timer;

    const auto timeout = std::chrono::seconds( 3 );

    int count = 0;

    while( true )
    {
        States prev = state;
        
        adas.read();

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
                        state = DONE;
                    adas.set_angle( 0.f );
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

        
        adas.write();

        std::this_thread::sleep_for( std::chrono::milliseconds(10) );
    }

	return 0;
}
