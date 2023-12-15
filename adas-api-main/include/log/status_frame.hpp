#ifndef LOG_STATUS_FRAME_HPP
#define LOG_STATUS_FRAME_HPP
#pragma once
#include "process_frame.hpp"

#include <candata.h>

#include <array>
#include <iostream>
#include <stdexcept>
#include <variant>


namespace adas_api{
    class LogStatusFrame : public ProcessFrame
    {
    public:
        enum State_ASSI
        {
            // CANDATA + message name + Signal name + Descr
            AUTOMATED_SYSTEM_OFF = CANDATA_VCU2_LOG_STATUS_STATE_ASSI_OFF_CHOICE,
            AUTOMATED_SYSTEM_READY = CANDATA_VCU2_LOG_STATUS_STATE_ASSI_READY_CHOICE,
            AUTOMATED_SYSTEM_DRIVING = CANDATA_VCU2_LOG_STATUS_STATE_ASSI_DRIVING_CHOICE,
            EMERGENCY_BRAKE = CANDATA_VCU2_LOG_STATUS_STATE_ASSI_EMERGENCY_BRAKE_CHOICE,
            AUTOMATED_SYSTEM_FINISHED = CANDATA_VCU2_LOG_STATUS_STATE_ASSI_FINISH_CHOICE
        };

        enum State_EBS
        {
            UNAVAILABLE = CANDATA_VCU2_LOG_STATUS_STATE_EBS_UNAVAILABLE_CHOICE,
            ARMED = CANDATA_VCU2_LOG_STATUS_STATE_EBS_ARMED_CHOICE,
            TRIGGERED = CANDATA_VCU2_LOG_STATUS_STATE_EBS_TRIGGERED_CHOICE
        };

        enum AMI_STATE
        {
            NOT_SELECTED = CANDATA_VCU2_LOG_STATUS_AMI_STATE_NOT_SELECTED_CHOICE, // name was NOT_ACCEPTED
            ACCELERATION = CANDATA_VCU2_LOG_STATUS_AMI_STATE_ACCELERATION_CHOICE,
            SKIDPAD = CANDATA_VCU2_LOG_STATUS_AMI_STATE_SKIDPAD_CHOICE,
            AUTOCROSS = CANDATA_VCU2_LOG_STATUS_AMI_STATE_AUTOCROSS_CHOICE,
            TRACK_DRIVE = CANDATA_VCU2_LOG_STATUS_AMI_STATE_TRACK_DRIVE_CHOICE,
            STATIC_INSPECTION_A = CANDATA_VCU2_LOG_STATUS_AMI_STATE_STATIC_INSPECTION_A_CHOICE,
            STATIC_INSPECTION_B = CANDATA_VCU2_LOG_STATUS_AMI_STATE_STATIC_INSPECTION_B_CHOICE,
            AUTONOMOUS_DEMO = CANDATA_VCU2_LOG_STATUS_AMI_STATE_AUTONOMOUS_DEMO_CHOICE,
        };

        enum State_Steering
        {
            STEERING_OFF = CANDATA_VCU2_LOG_STATUS_STATE_STEERING_OFF_CHOICE,
            STEERING_ACTIVE = CANDATA_VCU2_LOG_STATUS_STATE_STEERING_ON_CHOICE,
        };

        enum State_Service_Brake
        {
            DISENGAGED = CANDATA_VCU2_LOG_STATUS_STATE_SERVICE_BRAKE_DISENGAGED_CHOICE,
            ENGAGED = CANDATA_VCU2_LOG_STATUS_STATE_SERVICE_BRAKE_ENGAGED_CHOICE,
            AVAILABLE = CANDATA_VCU2_LOG_STATUS_STATE_SERVICE_BRAKE_AVAILABLE_CHOICE
        };

        const float lap_counter() const;
        const float cones_count_actual() const;
        const float cones_count_all() const;

        const State_ASSI status_indicator() const;
        const State_EBS braking_system() const;
        const AMI_STATE mission_indicator() const;
        const State_Steering steering_system() const;
        const State_Service_Brake serv_brake_status() const;

        LogStatusFrame();
        virtual ~LogStatusFrame() = default;

    private:
        virtual void _process( const can_frame &frame ) override;
        virtual void _print( std::ostream &os ) const override;

        candata_vcu2_log_status_t data;
    };
};

#endif // LOG_STATUS_FRAME_HPP
