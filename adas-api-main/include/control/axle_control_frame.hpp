#ifndef AXLE_CONTROL_FRAME_HPP
#define AXLE_CONTROL_FRAME_HPP
#pragma once
#include "status_frames.hpp"
#include "control_frames.hpp"

#include <candata.h>

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <variant>

namespace adas_api{
    class AxleControlFrame : public ControlFrame
    {
    public:
        bool is_front() const;

        AxleControlFrame( const AxleTorqueFrame &torqueFrame, 
                            const BrakeFrame &brakeFrame,
                            const WheelSpeedsFrame &speedsFrame, 
                            const StatusFrame &statusFrame,
                            const BrakeControlFrame &brakeControl,
                            const bool front=true );
        virtual ~AxleControlFrame() = default;

        void set_speed( const float speed );

        void set_torque( const float torque );

        void set_plaid( const bool plaid );

        /**
         * @brief Limit torque based on rpm.
         * 
         * Limit the torque based on the wheel rpm to help maintain constant electrical 
         * power and minimise the risk of an over-current trip. These limits are copied
         * from the official FS-AI_API code.
         * 
         * @param rpm 
         * @return float Limited torque.
         */
        float torque_rpm_limit( const float rpm ) const;

        constexpr static float motor_ratio = 3.5f;

    private:
        /**
         * @brief Generate can_frame for transmission.
         * 
         * @warning Will default to speed and torque = 0 unless all requirements are
         * met, i.e.:
         *  - Recent status frame.
         *  - Recent torque frame.
         *  - Recent wheel speeds frame.
         *  - AS_STATE == DRIVING
         * 
         * @return can_frame 
         */
        virtual can_frame _process() const override;
        virtual void _print( std::ostream& os ) const override;

        static float _plaid_mode( const float rpm );

        const AxleTorqueFrame &_torqueFrame;
        const BrakeFrame &_brakeFrame;
        const WheelSpeedsFrame &_speedsFrame;
        const StatusFrame &_statusFrame;
        const BrakeControlFrame &_brakeControl;

        const bool _front;
        float _speed = 0.f;
        float _torque = 0.f;
        bool _plaidMode = false;
    };
};


#endif // AXLE_CONTROL_FRAME_HPP
