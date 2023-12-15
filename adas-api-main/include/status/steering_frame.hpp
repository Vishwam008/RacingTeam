#ifndef STEERING_FRAME_HPP
#define STEERING_FRAME_HPP

#include "process_frame.hpp"

#include <candata.h>
#include <vector>

namespace adas_api{
    class SteeringFrame : public ProcessFrame
    {
    public:
        float angle() const;
        float requested() const;
        float max() const;

        void set_calibration( const std::vector<float>& calibration );

        /**
         * @brief Construct a new Steering Frame object
         * 
         * @param righthandrule Use right hand rule for steering angle.
         *                      I.e. positive angle is CCW.
         */
        SteeringFrame();
        virtual ~SteeringFrame() override = default;

    private:
        virtual void _process( const can_frame& frame ) override;
        virtual void _print( std::ostream& os ) const override;

        candata_vcu2_ai_steer_t data;

        std::vector<float> _calibration;
    };
};

#endif // STEERING_FRAME_HPP
