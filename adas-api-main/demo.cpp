#include "include/can_wrap.hpp"
#include "include/candata.h"

#include <iomanip>

int main()
{
    float f = 0.0E-1f;

    std::cout << std::fixed << f << "\n";

    return 0;

    const std::string device = "vcan0";
    const int canSocket = can::connect( device );

    while( true )
    {
        try
        {
            const can_frame frame = can::read( canSocket );

            if( frame.can_id == CANDATA_VCU2_AI_STATUS_FRAME_ID )
            {
                candata_vcu2_ai_status_t data;
                candata_vcu2_ai_status_unpack( &data, frame.data, frame.can_dlc );
                std::cout << "status: " << 
                    candata_vcu2_ai_status_as_state_decode( data.as_state ) << "\n";
            }
            else if( frame.can_id == CANDATA_VCU2_AI_BRAKE_FRAME_ID )
            {
                candata_vcu2_ai_brake_t data;
                candata_vcu2_ai_brake_unpack( &data, frame.data, frame.can_dlc );
                std::cout << "brake: " << std::fixed << std::setprecision( 10 ) <<
                    candata_vcu2_ai_brake_hyd_press_f_pct_decode( data.hyd_press_f_pct ) << " " <<
                    candata_vcu2_ai_brake_hyd_press_r_pct_decode( data.hyd_press_r_pct ) << "\n";
            }
            else if( frame.can_id == CANDATA_VCU2_AI_DRIVE_F_FRAME_ID )
            {
                candata_vcu2_ai_drive_f_t data;
                candata_vcu2_ai_drive_f_unpack( &data, frame.data, frame.can_dlc );
                std::cout << "axle: " <<
                    candata_vcu2_ai_drive_f_front_axle_trq_decode( data.front_axle_trq ) << "\n";
            }
               // std::cout << "status frame" << "\n";
        }
        catch( const can::error& )
        {
            // no more frames to read
        }
    }

    return 0;
}