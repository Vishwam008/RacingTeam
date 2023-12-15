#ifndef WHEEL_COUNTS_FRAME_HPP
#define WHEEL_COUNTS_FRAME_HPP
#pragma once
#include "process_frame.hpp"

#include <candata.h>

namespace adas_api{
    class WheelCountsFrame : public ProcessFrame
    {
    public:
        using counts_t = int;

        const counts_t count( size_t index ) const;
        const counts_t front_left() const;
        const counts_t front_right() const;
        const counts_t rear_left() const;
        const counts_t rear_right() const;

        WheelCountsFrame();
        ~WheelCountsFrame();

    private:
        virtual void _process( const can_frame& frame ) override;
        virtual void _print( std::ostream& os ) const override;

        candata_vcu2_ai_wheel_counts_t data;
    };
};

#endif // WHEEL_COUNTS_FRAME_HP
