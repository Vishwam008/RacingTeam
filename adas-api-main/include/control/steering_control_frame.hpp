#ifndef STEERING_CONTROL_FRAME_HPP
#define STEERING_CONTROL_FRAME_HPP

#include "control/control_frame.hpp"
#include "status/steering_frame.hpp"
#include "status/status_frame.hpp"

namespace adas_api{
    class SteeringControlFrame : public ControlFrame
    {
    public:
        /**
         * @brief Construct a new Steering Control Frame object
         * 
         * @remarks Will use the same direction rules as the steering frame.
         *      I.e. if righthandrule is set for steering frame then the 
         *      control frame will also use positive angles for CCW.
         * 
         * @param steeringFrame 
         * @param statusFrame 
         */
        SteeringControlFrame( const SteeringFrame &steeringFrame, const StatusFrame &statusFrame );
        virtual ~SteeringControlFrame() override = default;

        void set_angle( const float angle );

        void set_calibration( const std::vector<float>& calibration );

    private:
        /**
         * @brief Generate can_frame for transmission.
         * 
         * @warning Will default to angle = 0 unless all requirements are
         * met, i.e.:
         *  - Recent status frame.
         *  - Recent steering frame.
         *  - AS_STATE == DRIVING or AS_STATE == EMERGENCY
         * 
         * @return can_frame 
         */
        virtual can_frame _process() const override;
        virtual void _print( std::ostream& os ) const override;

        const SteeringFrame& _steeringFrame;
        const StatusFrame& _statusFrame;

        float _angle = 0.f;

        std::vector<float> _calibration;
    };
};

#endif // STEERING_CONTROL_FRAME_HPP
