#ifndef LOG_DYNAMICS1_HPP
#define LOG_DYNAMICS1_HPP
#pragma once
#include "process_frame.hpp"

#include <candata.h>

namespace adas_api{
    class LogDynamics1 : public ProcessFrame
    {
        const float speed_actual_kmh() const;
        const float speed_target_kmh() const;
        const float steer_actual_deg() const;
        const float steer_target_deg() const;
        const float brake_actual_pct() const;
        const float brake_target_pct() const;
        const float drive_trq_actual_pct() const;
        const float drive_trq_target_pct() const;

        LogDynamics1();
        ~LogDynamics1();

    private:
        virtual void _process(const can_frame& frame) override;
        virtual void _print( std::ostream& os ) const override;
    };
};

#endif // LOG_DYNAMICS1_HPP
