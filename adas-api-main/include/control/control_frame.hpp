#ifndef CONTROL_FRAME_HPP
#define CONTROL_FRAME_HPP
#pragma once
#include "can_wrap.hpp"

#include <chrono>
#include <iostream>

#include <linux/can.h>

namespace adas_api{
    class ControlFrame
    {
    public:
        ControlFrame( const uint32_t frameId );
        virtual ~ControlFrame() = default;
        const can_frame process();

        const uint32_t frameId;

        friend std::ostream& operator<<( std::ostream& os, const ControlFrame& p );

    private:
        /**
        * @brief  Generate can_frame for control
        * 
        * !TODO @warning  
        * 
        * @retval can_frame 
        */
        virtual can_frame _process() const = 0;
        virtual void _print( std::ostream& os ) const = 0;
    };

    std::ostream& operator<<( std::ostream& os, const ControlFrame& p )
    {
        p._print( os );
        return os;
    }
};

#endif // CONTROL_FRAME_HPP
