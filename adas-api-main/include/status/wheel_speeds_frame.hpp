#ifndef WHEEL_SPEEDS_FRAME_HPP
#define WHEEL_SPEEDS_FRAME_HPP
#pragma once
#include "process_frame.hpp"

#include <candata.h>

#include <array>
#include <stdexcept>

namespace adas_api{
    class WheelSpeedsFrame : public ProcessFrame
    {
    public:
        using rpm_t = int;

        const rpm_t rpm( size_t index ) const;
        const rpm_t front_left() const;
        const rpm_t front_right() const;
        const rpm_t rear_left() const;
        const rpm_t rear_right() const;
        const rpm_t fastest() const;
        const bool stopped() const;

        WheelSpeedsFrame();
        virtual ~WheelSpeedsFrame() = default;

    private:
        virtual void _process( const can_frame& frame ) override;
        virtual void _print( std::ostream& os ) const override;

        candata_vcu2_ai_speeds_t data;
    };
}

#endif // WHEEL_SPEEDS_FRAME_HPP
