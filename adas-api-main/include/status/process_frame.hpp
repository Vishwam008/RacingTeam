#ifndef PROCESS_FRAME_HPP
#define PROCESS_FRAME_HPP
#pragma once
#include <chrono>
#include <iostream>

#include <linux/can.h>

namespace adas_api{
    class ProcessFrame
    {
    public:
        ProcessFrame( const uint32_t frameId );
        virtual ~ProcessFrame() = default;
        void process( const can_frame& frame );
        bool is_recent() const;

        using timestamp_t = std::chrono::steady_clock::time_point;

        const uint32_t frameId;
        const timestamp_t& timestamp() const;

        friend std::ostream& operator<<( std::ostream& os, const ProcessFrame& p );

    private:
        timestamp_t _timestamp;
        const std::chrono::milliseconds _timeout = std::chrono::milliseconds( 100 );

        virtual void _process( const can_frame& frame ) = 0;
        virtual void _print( std::ostream& os ) const = 0;
    };

    std::ostream& operator<<( std::ostream& os, const ProcessFrame& p )
    {
        p._print( os );
        return os;
    }
};

#endif // PROCESS_FRAME_HPP
