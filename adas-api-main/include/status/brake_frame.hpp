#ifndef BRAKE_FRAME_HPP
#define BRAKE_FRAME_HPP
#pragma once
#include "process_frame.hpp"

#include <candata.h>

#include <array>
#include <iostream>
#include <stdexcept>
#include <variant>

namespace adas_api{
    class BrakeFrame : public ProcessFrame
    {
    public:
        enum BrakeState
        {
            INITIALISING = CANDATA_VCU2_AI_BRAKE_STATUS_BRK_INITIALISING_CHOICE,
            READY = CANDATA_VCU2_AI_BRAKE_STATUS_BRK_READY_CHOICE,
            SHUTTING_DOWN = CANDATA_VCU2_AI_BRAKE_STATUS_BRK_SHUTTING_DOWN_CHOICE,
            SHUTDOWN_COMPLETE = CANDATA_VCU2_AI_BRAKE_STATUS_BRK_SHUTDOWN_COMPLETE_CHOICE,
            FAULT = CANDATA_VCU2_AI_BRAKE_STATUS_BRK_FAULT_CHOICE
        };

        enum EbsState
        {
            UNAVAILABLE = CANDATA_VCU2_AI_BRAKE_STATUS_EBS_UNAVAILABLE_CHOICE,
            ARMED = CANDATA_VCU2_AI_BRAKE_STATUS_EBS_ARMED_CHOICE,
            TRIGGERED = CANDATA_VCU2_AI_BRAKE_STATUS_EBS_TRIGGERED_CHOICE
        };

        const float front_brake() const;
        const float front_brake_requested() const;
        const float rear_brake() const;
        const float rear_brake_requested() const;

        const bool front_on() const { return std::abs( 0.f - this->front_brake() ) > std::numeric_limits<float>::epsilon(); }
        const bool rear_on() const { return std::abs( 0.f - this->rear_brake() ) > std::numeric_limits<float>::epsilon(); }
        const bool on() const { return this->front_on() || this->rear_on(); }

        const BrakeState brake_state() const;

        const EbsState ebs_state() const;

        BrakeFrame();
        virtual ~BrakeFrame() = default;

    private:
        virtual void _process( const can_frame &frame ) override;
        virtual void _print( std::ostream &os ) const override;

        candata_vcu2_ai_brake_t data;
    };
};

#endif // BRAKE_FRAME_HPP
