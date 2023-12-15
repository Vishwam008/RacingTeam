#ifndef BRAKE_CONTROL_FRAME_HPP
#define BRAKE_CONTROL_FRAME_HPP
#pragma once
#include "control/control_frame.hpp"

namespace adas_api{
    class BrakeControlFrame : public ControlFrame
    {
    public:
        BrakeControlFrame();
        virtual ~BrakeControlFrame() override = default;

        void set_front( const float percentage );
        void set_rear( const float percentage );
        void set_brakes( const float percentage );

        float get_front() const;
        float get_rear() const;
        bool on() const;

    private:
        /**
        * @brief  Generate can_frame for brake
        * 
        * !TODO @warning  
        * 
        * @retval can_frame
        */
        virtual can_frame _process() const override;
        virtual void _print( std::ostream& os ) const override;

        float _front = 0.f;
        float _rear = 0.f;
    };
};

#endif // BRAKE_CONTROL_FRAME_HPP
