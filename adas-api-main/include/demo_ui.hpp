#ifndef DEMO_UI_HPP
#define DEMO_UI_HPP

#include "ftxui/component/captured_mouse.hpp"  // for ftxui
#include "ftxui/component/component.hpp"  // for Menu, Renderer, Horizontal, Vertical
#include "ftxui/component/component_base.hpp"  // for ComponentBase
#include "ftxui/component/screen_interactive.hpp"  // for Component, ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for text, Element, operator|, window, flex, vbox

#include <sstream>

#include <adas.hpp>

using namespace ftxui;

namespace demo_ui
{
    std::string device;

    struct MenuConfig
    {
        std::vector<std::string> states;
        int opt = 0;

        ftxui::Component Menu( bool vertical = true )
        {
            return ftxui::Menu( &states, &opt, 
                    vertical ? 
                    ftxui::MenuOption::VerticalAnimated() :
                    ftxui::MenuOption::HorizontalAnimated() );
        }
    };

    struct SliderConfig
    {
        const float scaling = 1;
        const float limit = 1000;
        const float minimum = 0;
        int value = 0;
        std::string input;

        void reset() { value = 0; input.clear(); }
        float scaledValue() const { return static_cast<float>(value) / scaling;}
        ftxui::Component Slider() 
        {
            return ftxui::Slider( "", 
                        &(value), 
                        static_cast<int>( scaling*minimum ), 
                        static_cast<int>( scaling*limit ), 
                        static_cast<int>( scaling ) );
        }
    };

    std::string errorMessages;

    template <typename T>
    std::string to_string_with_precision(const T a_value, const int n = 6)
    {
        std::ostringstream out;
        out.precision(n);
        out << std::fixed << a_value;
        return out.str();
    }

    inline float deg2Rad( float degree )
    {
        static const float pi = 3.14159265359f;
        return ( degree * ( pi / 180 ) );
    }

    inline float rad2Deg( float radian )
    {
        static const double pi = 3.14159265359f;
        return ( radian *  ( 180 / pi ) );
    }

    MenuConfig missionMenu { .states = { "NOT SELECTED", "SELECTED", "RUNNING", "FINISHED" } };
    MenuConfig angleMenu { .states = { "DEGREES", "RADIANS" } };
    MenuConfig directionMenu { .states = { "LEFTHAND", "RIGHTHAND" } };
    MenuConfig uiMenu { .states = { "SINGLE", "DUAL" } };
    MenuConfig driveMenu { .states = { "NEUTRAL", "DRIVE" } };
    MenuConfig ebrakeMenu { .states = { "OFF", "ON" } };

    std::vector<std::string> switchStates { "OFF", "ON" };
    
    SliderConfig frontRpm { .scaling = 1000, .limit = 1000 };
    SliderConfig rearRpm  { .scaling = 1000, .limit = 1000 };
    SliderConfig frontTorque  { .scaling = 10000, .limit = 200 };
    SliderConfig rearTorque { .scaling = 10000, .limit = 200 };
    SliderConfig frontBrake { .scaling = 100, .limit = 100 };
    SliderConfig rearBrake { .scaling = 100, .limit = 100 };
    SliderConfig steering { .scaling = 10000, .limit = 21.f, .minimum = -21.f };
    SliderConfig steeringRads { .scaling = 10000, .limit = deg2Rad(steering.limit), .minimum = deg2Rad(steering.minimum) };


    ftxui::Component Window( std::string title, Component component )
    {
        return Renderer(component, [component, title] {  //
            return window(text(title), component->Render()) | flex;
        });
    }

    ftxui::Component status_display( adas_api::Adas &interface )
    {
        auto as_state = [&]( adas_api::StatusFrame::AsState state )
            {
                switch( state )
                {
                    case adas_api::StatusFrame::AsState::OFF: return "OFF";
                    case adas_api::StatusFrame::AsState::READY: return "READY";
                    case adas_api::StatusFrame::AsState::DRIVING: return "DRIVING";
                    case adas_api::StatusFrame::AsState::EMERGENCY: return "EMERGENCY";
                    case adas_api::StatusFrame::AsState::FINISHED: return "FINISHED";
                }

                return "ERROR";
            };

        auto ami_state = [&]( adas_api::StatusFrame::AmiState state )
            {
                switch( state )
                {
                    case adas_api::StatusFrame::AmiState::ACCELERATION: return "ACCELERATION";
                    case adas_api::StatusFrame::AmiState::SKIDPAD: return "SKIDPAD";
                    case adas_api::StatusFrame::AmiState::AUTOCROSS: return "AUTOCROSS";
                    case adas_api::StatusFrame::AmiState::TRACK_DRIVE: return "TRACK_DRIVE";
                    case adas_api::StatusFrame::AmiState::STATIC_INSPECTION_A: return "STATIC_INSPECTION_A";
                    case adas_api::StatusFrame::AmiState::STATIC_INSPECTION_B: return "STATIC_INSPECTION_B";
                    case adas_api::StatusFrame::AmiState::AUTONOMOUS_DEMO: return "AUTONOMOUS_DEMO";
                }

                return "ERROR";
            };

        auto status = []( const std::string s, const bool recent )
            {
                return hbox(
                    {
                        text( s ), 
                        filler(), 
                        text( recent ? "OK" : "NO" ) | color( recent ? Color::Green : Color::Red )
                    } );
            };

        return Container::Vertical(
            {
                Window( "Warnings", Renderer( [&]
                    {
                        return paragraph( interface.status.warning_message() );
                    } ) ) | yflex_grow,
                Window( "Faults", Renderer( [&]
                    {
                        return paragraph( interface.status.fault_message() );
                    } ) ) | yflex_grow,
                Window( "Status", Renderer( [&]
                    {
                        return vbox(
                            {
                                hbox({text("AS  "),filler(),text(as_state(interface.status.as_state()))}),
                                hbox({text("AMI "),filler(),text(ami_state(interface.status.ami_state()))}),
                                hbox({text("Go "),filler(),text(interface.status.go() ? "GO" : "NO GO")|color( interface.status.go() ? Color::Green : Color::Red)}),
                                hbox({text("Handshake "),filler(),text(interface.status.handshake()?"●":"○")}),
                                hbox({text("Device "),filler(),text(device)}),
                            });
                    } ) ) | yflex_shrink,
                Window( "Status frames", Renderer( [&]()
                    {
                        return vbox(
                            {
                                status("Front axle ", interface.frontAxle.is_recent() ),
                                status("Rear axle ", interface.rearAxle.is_recent() ),
                                status("Brake ", interface.brakes.is_recent() ),
                                status("Status ", interface.status.is_recent() ),
                                status("Steering ", interface.steering.is_recent() ),
                                status("Wheel counts ", interface.wheelCounts.is_recent() ),
                                status("Wheel speeds", interface.wheelSpeeds.is_recent() ),
                            } );
                    } ) ) | yflex_shrink,
            } );
    }

    /**
     * @brief Generic slider controller with manual input box and value readout.
     * 
     * @param config 
     * @return ftxui::Component 
     */
    static ftxui::Component generic_control( SliderConfig &config )
    {
        return Container::Vertical(
            {
                Slider( "", &config.value, static_cast<int>( config.minimum*config.scaling ), 
                        static_cast<int>( config.limit*config.scaling ), 1 ),
                Renderer( [&]()
                    {
                        return hbox(
                            {
                                text( to_string_with_precision( config.minimum, 2 ) ),
                                filler(),
                                text( config.minimum != 0 ? "^" : "" ),
                                filler(),
                                text( to_string_with_precision( config.limit, 2 ) )
                            } );
                    } ),
                Renderer( [&]()
                    {
                        return hbox(
                            {
                                text( "Value: " ),
                                text( to_string_with_precision( config.value / config.scaling, 1 ) )
                            } );
                    } ),
                Container::Horizontal(
                    {
                        Renderer( [](){ return text("Manual input: "); } ),
                        Input( &config.input, "",
                            InputOption 
                            { 
                                .on_enter = [&]()
                                    {
                                        try
                                        {
                                            const float val = std::max( config.minimum, 
                                                                std::min( config.limit, 
                                                                std::stof( config.input ) ) );
                                            config.value = static_cast<int>( val * config.scaling );
                                        }
                                        catch (...)
                                        {
                                            // invalid input
                                        }

                                        config.input = ""; 
                                    } 
                            } ) | xflex_grow | bgcolor(Color::Magenta),
                    } )
            } );
    }

    /**
     * @brief Create a dual slider control with front/rear labels.
     * 
     * @param opt 
     * @param fConfig 
     * @param rConfig 
     * @return ftxui::Component 
     */
    static ftxui::Component dual_control( int &opt, SliderConfig &fConfig, SliderConfig &rConfig )
    {
        return Container::Horizontal(
            {
                Container::Vertical(
                    {
                        Renderer( [](){ return text("Front"); } ) | Maybe( [&](){ return opt != 0; } ),
                        generic_control( fConfig )
                    } ) | xflex_grow,
                Renderer( [](){ return separator(); } ) | Maybe( [&](){ return opt != 0; } ),
                Container::Vertical(
                    {
                        Renderer( [](){ return text("Rear"); } ),
                        generic_control( rConfig )
                    } ) | xflex_grow | Maybe( [&](){ return opt != 0; } ),
            } );
    }

    /**
     * @brief Create the vehicle controls region
     * 
     * @return ftxui::Component 
     */
    ftxui::Component state_controls()
    {
        return Container::Vertical(
            {
                Container::Horizontal(
                    {   Window( "Mission status", missionMenu.Menu() ),
                        Window( "Ebrake", ebrakeMenu.Menu() ),
                        Window( "Drive", driveMenu.Menu() ),
                        Window( "UI mode", uiMenu.Menu() ),
                    } ) | yflex_shrink,
                Window( "Rpm", dual_control( uiMenu.opt, frontRpm, rearRpm ) ), //  | x 0.09 km/h x 5/18 to get m/s
                Window( "Torque", dual_control( uiMenu.opt, frontTorque, rearTorque ) ),
                Window( "Brake", dual_control( uiMenu.opt, frontBrake, rearBrake ) ),
                Window( "Steering", 
                    Container::Vertical(
                        {
                            Container::Horizontal(
                                {   
                                    angleMenu.Menu( false ),
                                    Renderer( [](){ return filler(); } ),
                                    directionMenu.Menu( false )
                                } ),
                            generic_control( steering ) | Maybe( [&](){ return angleMenu.opt == 0; } ),
                            generic_control( steeringRads ) | Maybe( [&](){ return angleMenu.opt == 1; } ),
                        } ) ),
            } );
    }

    ftxui::Element fmt_float( const float val )
    { 
        return text(to_string_with_precision(val,1)) | 
                    align_right | size( WIDTH, GREATER_THAN, 6 );
    }

    ftxui::Element fmt_int( const int val )
    { 
        return text(std::to_string(val)) | 
                    align_right | size( WIDTH, GREATER_THAN, 5 ); 
    }

    ftxui::Element wheel_display( int count, int rpm )
    {
        return window(text("Wheel"),vbox(
            {
                hbox({text("Count "), fmt_int( count )}),
                hbox({text("Rpm   "), fmt_int( rpm )}),
            } ) );
    }

    ftxui::Element axle_display( float current, float max, float requested )
    {
        return window(text("Axle (Nm)"),vbox(
            {
                hbox({text("Crrnt "), fmt_float(current)}),
                hbox({text("Max   "), fmt_float(max)}),
                hbox({text("Rqstd "), fmt_float(requested)}),
            } ) );
    }

    ftxui::Element brake_display( float brake, float requested )
    {
        return window(text("Brake (%)"),vbox(
            {
                hbox({text("Brake "), fmt_float(brake)}),
                hbox({text("Rqstd "), fmt_float(requested)}),
            } ) );
    }

    ftxui::Element chassis_display( bool as, bool ts, bool ebrake )
    {
        return vbox(
            {
                hbox({text("AS     "), filler(), text(as?"ON ":"OFF") | color( as? Color::Green : Color::Red)}),
                hbox({text("TS     "), filler(), text(ts?"ON ":"OFF") | color( ts? Color::Green : Color::Red)}),
                hbox({text("Ebrake "), filler(), text(ebrake?"ON ":"OFF") | color( ebrake? Color::Red : Color::Green)}),
            } ) | border;
    }

    ftxui::Element steer_display( float actual, float max, float requested )
    {
        return window(text("Steering"),vbox(
            {
                hbox({text("Actual "), 
                        text(to_string_with_precision( angleMenu.opt ? deg2Rad( actual ) : actual, 2 ))}),
                hbox({text("Max    "), 
                        text(to_string_with_precision( angleMenu.opt ? deg2Rad( max ) : max, 2 ))}),
                hbox({text("Reqstd "), 
                        text(to_string_with_precision( angleMenu.opt ? deg2Rad( requested ) : requested, 2 ))}),
            } ) );
    }

    ftxui::Element car_display( adas_api::Adas &interface )
    {
        return gridbox(
            {
                {
                    filler(),
                    steer_display( interface.steering.angle(), 
                            interface.steering.max(), 
                            interface.steering.requested() ),
                    filler()
                },
                {
                    wheel_display( interface.wheelCounts.front_left(), 
                            interface.wheelSpeeds.front_left() ),
                    vcenter( hcenter( text("Front") ) ),
                    wheel_display( interface.wheelCounts.front_right(), 
                            interface.wheelSpeeds.front_right() ),
                },
                {
                    filler(),
                    gridbox(
                        {
                            {
                                axle_display( interface.frontAxle.get_current(), 
                                        interface.frontAxle.get_max(),
                                        interface.frontAxle.get_requested() ),
                                brake_display( interface.brakes.front_brake(), 
                                        interface.brakes.front_brake_requested() ),
                            },
                            {
                                chassis_display( interface.status.as_switch(),
                                        interface.status.ts_switch(),
                                        false ),
                            },
                            {
                                axle_display( interface.rearAxle.get_current(), 
                                        interface.rearAxle.get_max(),
                                        interface.rearAxle.get_requested() ),
                                brake_display( interface.brakes.rear_brake(), 
                                        interface.brakes.rear_brake_requested() ),
                            }
                        } ),
                    filler(),
                },
                {
                    wheel_display( interface.wheelCounts.rear_left(), 
                            interface.wheelSpeeds.rear_left() ),
                    vcenter( hcenter( text("Rear") ) ),
                    wheel_display( interface.wheelCounts.rear_right(), 
                            interface.wheelSpeeds.rear_right() ),
                }, 
            } );
    }

    ftxui::Element _credits()
    {
        return paragraph("Phoenix Racing, Coventry University. 2023.") | center | bold | color( Color::BlueLight);
    }


    ftxui::Component layout( adas_api::Adas &interface )
    {
        return Container::Horizontal(
            {
                Container::Vertical(
                    {
                        Window( "Status", Container::Horizontal(
                            {
                                status_display( interface ),
                                Renderer( [&]()
                                    {
                                        return vbox({
                                            _credits(),
                                            filler(),
                                            car_display( interface ),
                                            filler(),
                                            paragraph( errorMessages )
                                        });
                                    } ),
                            } ) ),
                        Container::Horizontal(
                            {
                                Button( "EMERGENCY STOP", [&](){ ebrakeMenu.opt = 1; } ) | 
                                            bgcolor(Color::Red) | xflex_grow,
                                Button( "RESET", [&]()
                                    { 
                                        ebrakeMenu.opt = 0; 
                                        driveMenu.opt = 0;
                                        steering.reset();
                                        frontRpm.reset();
                                        rearRpm.reset();
                                        frontTorque.reset();
                                        rearTorque.reset();
                                        frontBrake.reset();
                                        rearBrake.reset();
                                        missionMenu.opt = 0;

                                    } ) | bgcolor(Color::Blue) | xflex_shrink,
                            }
                        )
                    } ),
                state_controls() | xflex_grow,
            } );
    }



};


#endif // DEMO_UI_HPP
