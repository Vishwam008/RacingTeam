#include "simple_adas.hpp"

#include <can_wrap.hpp>

#include <thread>

bool adas_api::SimpleAdas::is_recent() const
{
    return this->front_axle().is_recent() && 
            this->rear_axle().is_recent() && 
            this->brakes().is_recent() && 
            this->status().is_recent() && 
            this->steering().is_recent() && 
            this->wheel_counts().is_recent() && 
            this->wheel_speeds().is_recent(); 
}

void adas_api::SimpleAdas::set_steering_request_calibration( const std::vector<float>& calibration )
{
    this->steeringControl.set_calibration( calibration );
}

void adas_api::SimpleAdas::set_steering_feedback_calibration( const std::vector<float>& calibration )
{
    static_cast<adas_api::Adas*>(this)->steering.set_calibration( calibration );
}

void adas_api::SimpleAdas::set_front_rpm( const float rpm )
{
    _set_rpm( this->frontControl, rpm );
}

void adas_api::SimpleAdas::set_rear_rpm( const float rpm )
{
    _set_rpm( this->rearControl, rpm );
}

void adas_api::SimpleAdas::set_rpm( const float rpm )
{
    this->set_front_rpm( rpm );
    this->set_rear_rpm( rpm );
}

void adas_api::SimpleAdas::set_plaid( const bool plaid )
{
    this->frontControl.set_plaid( plaid );
    this->rearControl.set_plaid( plaid );
}

void adas_api::SimpleAdas::_set_rpm( AxleControlFrame &axle, const float rpm )
{
    axle.set_speed( rpm );
    axle.set_torque( std::fabs( rpm - 0.f ) < std::numeric_limits<float>::epsilon() ?
        0 : 200 );
}

void adas_api::SimpleAdas::set_front_brake( const float percentage )
{
    this->brakeControl.set_front( percentage );
}

void adas_api::SimpleAdas::set_rear_brake( const float percentage )
{
    this->brakeControl.set_rear( percentage );
}

void adas_api::SimpleAdas::set_brakes( const float percentage )
{
    this->brakeControl.set_brakes( percentage );
}

void adas_api::SimpleAdas::set_angle( const float degrees )
{
    this->steeringControl.set_angle( degrees );
}

bool adas_api::SimpleAdas::go() const 
{ 
    return this->status().go(); 
}

void adas_api::SimpleAdas::finish()
{
    this->statusControl.set_mission_status(
        StatusControlFrame::MissionStatus::FINISHED );
}

void adas_api::SimpleAdas::ebrake() 
{ 
    this->statusControl.set_estop( true ); 
}

const adas_api::AxleTorqueFrame &adas_api::SimpleAdas::front_axle() const 
{ 
    return Adas::frontAxle; 
}
const adas_api::AxleTorqueFrame &adas_api::SimpleAdas::rear_axle() const 
{ 
    return Adas::rearAxle; 
}

const adas_api::BrakeFrame &adas_api::SimpleAdas::brakes() const
{
    return Adas::brakes;
}

const adas_api::StatusFrame &adas_api::SimpleAdas::status() const
{
    return Adas::status;
}

const adas_api::SteeringFrame &adas_api::SimpleAdas::steering() const
{
    return Adas::steering;
}

const adas_api::WheelCountsFrame &adas_api::SimpleAdas::wheel_counts() const 
{ 
    return Adas::wheelCounts; 
}

const adas_api::WheelSpeedsFrame &adas_api::SimpleAdas::wheel_speeds() const 
{ 
    return Adas::wheelSpeeds; 
}

void adas_api::SimpleAdas::write()
{
    /*
        if AS OFF
            if AMI NOT_SELECTED
                - mission status: NOT_SELECTED
                - drive: false
                - estop: false
            else
                - mission status: SELECTED
                - drive: false
                - estop: false
        AS READY
            - mission status SELECTED
            - drive: false
            - estop: false
        AS DRIVING
            if GO NO_GO
                - mission status: SELECTED
                - drive: false
                - estop: false
            else
                - mission status: RUNNING
                - drive: true
                - estop: false
        AS FINISHED
            - mission status: FINISHED
            - drive: false
            - estop: false

        AS EMERGENCY
            - mission status: FINISHED
            - drive: false
            - estop: true
    */

   switch( this->status().as_state() )
    {
        case StatusFrame::AsState::OFF: 
            //std::cout << "AS OFF" << std::endl;
            if( this->status().ami_state() >= StatusFrame::AmiState::ACCELERATION && 
                this->status().ami_state() <= StatusFrame::AmiState::AUTONOMOUS_DEMO )
            {
                this->statusControl.set_mission_status( StatusControlFrame::MissionStatus::SELECTED );
            }
            else
            {
                this->statusControl.set_mission_status( StatusControlFrame::MissionStatus::NOT_SELECTED );
            }
            this->statusControl.set_drive( false );
            //this->statusControl.set_estop( false );
            break;
        case StatusFrame::AsState::READY:
            //std::cout << "AS READY" << std::endl;
            this->statusControl.set_mission_status( StatusControlFrame::MissionStatus::SELECTED );
            this->statusControl.set_drive( false );
            //this->statusControl.set_estop( false );
            break;
        case StatusFrame::AsState::DRIVING:
            //std::cout << "AS DRIVING" << std::endl;
            if( this->statusControl.get_mission_status() != StatusControlFrame::MissionStatus::FINISHED )
                this->statusControl.set_mission_status( StatusControlFrame::MissionStatus::SELECTED );
            this->statusControl.set_drive( true );
            //this->statusControl.set_estop( false );
            break;
        case StatusFrame::AsState::FINISHED:
            //std::cout << "AS FINISHED" << std::endl;
            this->statusControl.set_mission_status( StatusControlFrame::MissionStatus::NOT_SELECTED );
            this->statusControl.set_drive( false );
            //this->statusControl.set_estop( false );
            break;
        case StatusFrame::AsState::EMERGENCY:
            //std::cout << "AS EMERGENCY" << std::endl;
            this->statusControl.set_mission_status( StatusControlFrame::MissionStatus::NOT_SELECTED );
            this->statusControl.set_drive( false );
            //this->statusControl.set_estop( true );
            break;
        default: 
            //std::cout << "AS default" << std::endl;
            this->statusControl.set_mission_status( StatusControlFrame::MissionStatus::NOT_SELECTED );
            this->statusControl.set_drive( false );
            //this->statusControl.set_estop( false );
            break;
    }

    Adas::write();

    this->statusControl.set_estop( false );
}

