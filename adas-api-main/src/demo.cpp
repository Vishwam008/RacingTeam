#include <algorithm>
#include <thread>
#include <stdlib.h>
#include <fcntl.h>

#include "demo_ui.hpp"

int main( int argc, char *argv[] ) 
{
    demo_ui::device = argc >= 2 ? argv[1] : "can0";
    adas_api::Adas interface( demo_ui::device );

    
    auto screen = ftxui::ScreenInteractive::Fullscreen();

    /* We need a way to have the CAN frame code and the UI running at the same
        time. They need access to the same data so I to mutex or atomic everything.
        Or the UI Post() method is thread safe. 
        So, simple option. Events are triggered but a background thread with some 
        timers and all CAN code is handled by the UI as a series of events. */
    bool eventThreadRunning = true;
    auto readCan = [&]()
    {
        try
        {
            for( int i=0; i<50; ++i )
                interface.read();
        }
        catch( const std::runtime_error& e )
        {
            // no more frames to read
        }
    };

    auto writeCan = [&]()
    {
        demo_ui::errorMessages.clear();

        try
        {
            interface.frontControl.set_speed( demo_ui::frontRpm.scaledValue() );
            interface.rearControl.set_speed( demo_ui::uiMenu.opt == 0 ? 
                                             demo_ui::frontRpm.scaledValue() :
                                             demo_ui::rearRpm.scaledValue() );

            interface.frontControl.set_torque( demo_ui::frontTorque.scaledValue() );
            interface.rearControl.set_torque( demo_ui::uiMenu.opt == 0 ?
                                              demo_ui::frontTorque.scaledValue() :
                                              demo_ui::rearTorque.scaledValue() );

            interface.brakeControl.set_front( demo_ui::frontBrake.scaledValue() );
            interface.brakeControl.set_rear( demo_ui::uiMenu.opt == 0 ?
                                             demo_ui::frontBrake.scaledValue() :
                                             demo_ui::rearBrake.scaledValue() );

            interface.statusControl.set_drive( demo_ui::driveMenu.opt );
            interface.statusControl.set_estop( demo_ui::ebrakeMenu.opt );
            interface.statusControl.set_mission_status( static_cast<adas_api::StatusControlFrame::MissionStatus>( demo_ui::missionMenu.opt ) );

            interface.steeringControl.set_angle( demo_ui::angleMenu.opt == 0 ?
                                                 demo_ui::steering.scaledValue() :
                                                 demo_ui::rad2Deg( demo_ui::steeringRads.scaledValue() ) );

            interface.write();
        }
        catch( const std::runtime_error& e )
        {
            demo_ui::errorMessages = e.what();
        }
    };

    std::thread eventTriggerThread = std::thread( [&]
        {
            auto now = std::chrono::steady_clock::now();
            auto lastRead = now;
            auto lastWrite = now;
            auto lastUpdate = now;

            /* must not start posting events to the screen until it is in it's 
                Loop(), we have to start this thread before we start the Loop()
                so just pause here and give the screen half a sec to get going */
            std::this_thread::sleep_for( std::chrono::milliseconds(500) );

            while( eventThreadRunning )
            {
                now = std::chrono::steady_clock::now();

                if( now - lastRead >= std::chrono::milliseconds( 10 ) )
                {
                    screen.Post( readCan );
                    lastRead = now;
                }

                if( now - lastWrite >= std::chrono::milliseconds( 10 ) )
                {
                    screen.Post( writeCan );
                    lastWrite = now;
                }

                if( now - lastUpdate >= std::chrono::milliseconds( 1000/24 ) )
                {
                    screen.Post( ftxui::Event::Custom );
                    lastUpdate = now;
                }

                std::this_thread::sleep_for( std::chrono::milliseconds(5) );
            }
        } ); 

    screen.Loop( demo_ui::layout( interface ) );

    eventThreadRunning = false;
	eventTriggerThread.join();

    return 0;
}
