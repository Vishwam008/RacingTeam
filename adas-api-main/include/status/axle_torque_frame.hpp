#ifndef AXLE_TORQUE_FRAME_HPP
#define AXLE_TORQUE_FRAME_HPP
#pragma once 
#include "process_frame.hpp"

#include "candata.h"

#include <array>
#include <iostream>
#include <stdexcept>
#include <variant>

namespace adas_api{
    class AxleTorqueFrame : public ProcessFrame
    {
    public:
        float get_current() const;
        float get_max() const;
        float get_requested() const;
        bool is_front() const;
        bool stopped() const;

        AxleTorqueFrame( const bool front=true );
        virtual ~AxleTorqueFrame() = default;

    private:
        virtual void _process( const can_frame& frame ) override;
        virtual void _print( std::ostream& os ) const override;

        std::variant<candata_vcu2_ai_drive_f_t, candata_vcu2_ai_drive_r_t> data;
    };
};

#endif // AXLE_TORQUE_FRAME_HPP
