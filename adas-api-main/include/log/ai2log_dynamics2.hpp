#ifndef A12LOG_DYNAMICS2_HPP
#define A12LOG_DYNAMICS2_HPP
#pragma once
#include "../status/process_frame.hpp"

#include <candata.h>

namespace adas_api{
    class Ai2LogDynamics2 : public ProcessFrame
    {
    public:

        float accel_longitudinal_ms2() const;
        float accel_lateral_mps2() const;
        float yaw_rate_degps() const;

        Ai2LogDynamics2();

        ~Ai2LogDynamics2();

    private:

        virtual void _process(const can_frame& frame) override;
        virtual void _print( std::ostream& os ) const override;
    };
};

#endif // A12LOG_DYNAMICS2_HPP
