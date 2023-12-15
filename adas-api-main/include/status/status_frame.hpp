#ifndef STATUS_FRAME_HPP
#define STATUS_FRAME_HPP
#pragma once
#include "process_frame.hpp"

#include <candata.h>

#include <array>
#include <stdexcept>

namespace adas_api{
    class StatusFrame : public ProcessFrame
    {
    public:
        enum AsState
        {
            OFF = CANDATA_VCU2_AI_STATUS_AS_STATE_AS_OFF_CHOICE,
            READY = CANDATA_VCU2_AI_STATUS_AS_STATE_AS_READY_CHOICE,
            DRIVING = CANDATA_VCU2_AI_STATUS_AS_STATE_AS_DRIVING_CHOICE,
            EMERGENCY = CANDATA_VCU2_AI_STATUS_AS_STATE_EMERGENCY_BRAKE_CHOICE,
            FINISHED = CANDATA_VCU2_AI_STATUS_AS_STATE_AS_FINISHED_CHOICE
        };

        enum AmiState
        {
            NOT_SELECTED = CANDATA_VCU2_AI_STATUS_AMI_STATE_NOT_SELECTED_CHOICE,
            ACCELERATION = CANDATA_VCU2_AI_STATUS_AMI_STATE_ACCELERATION_CHOICE,
            SKIDPAD = CANDATA_VCU2_AI_STATUS_AMI_STATE_SKIDPAD_CHOICE,
            AUTOCROSS =  CANDATA_VCU2_AI_STATUS_AMI_STATE_AUTOCROSS_CHOICE,
            TRACK_DRIVE =  CANDATA_VCU2_AI_STATUS_AMI_STATE_TRACK_DRIVE_CHOICE,
            STATIC_INSPECTION_A = CANDATA_VCU2_AI_STATUS_AMI_STATE_STATIC_INSPECTION_A_CHOICE,
            STATIC_INSPECTION_B = CANDATA_VCU2_AI_STATUS_AMI_STATE_STATIC_INSPECTION_B_CHOICE,
            AUTONOMOUS_DEMO =  CANDATA_VCU2_AI_STATUS_AMI_STATE_AUTONOMOUS_DEMO_CHOICE
        };

        StatusFrame();
        virtual ~StatusFrame() = default;

        bool handshake() const;

        bool shutdown() const
        {
            return candata_vcu2_ai_status_shutdown_request_decode( data.shutdown_request ) 
                == CANDATA_VCU2_AI_STATUS_SHUTDOWN_REQUEST_SHUTDOWN_REQUESTED_CHOICE;
        }

        bool as_switch() const
        {
            return candata_vcu2_ai_status_as_switch_status_decode( data.as_switch_status ) 
                == CANDATA_VCU2_AI_STATUS_AS_SWITCH_STATUS_ON_CHOICE;
        }

        bool ts_switch() const
        {
            return candata_vcu2_ai_status_ts_switch_status_decode( data.ts_switch_status ) 
                == CANDATA_VCU2_AI_STATUS_TS_SWITCH_STATUS_ON_CHOICE;
        }

        /**
         * @brief 
         * 
         * @warning Does not appear to be used.
         */
        bool go() const
        {
            return candata_vcu2_ai_status_go_signal_decode( data.go_signal )
                == CANDATA_VCU2_AI_STATUS_GO_SIGNAL_GO_CHOICE;
        }

        bool steering() const
        {
            return candata_vcu2_ai_status_steering_status_decode( data.steering_status )
                == CANDATA_VCU2_AI_STATUS_STEERING_STATUS_ACTIVE_CHOICE;
        }

        AsState as_state() const
        {
            return static_cast<AsState>( candata_vcu2_ai_status_as_state_decode( data.as_state ) );
        }

        AmiState ami_state() const
        {
            return static_cast<AmiState>( candata_vcu2_ai_status_ami_state_decode( data.ami_state ) );
        }

        /**
         * @brief One of the fault flags has been set. 
         */
        bool fault() const;

        /**
         * @brief Generate a meaningful message listing all the warnings.
         */
        const std::string fault_message() const;

        /**
         * @brief One of the warning flags has been set.
         */
        bool warning() const;

        /**
         * @brief Generate a meaningful message listing all the faults.
         */
        const std::string warning_message() const;

        /**
         * @brief Check for warning status flag.
         */
        bool warning_status() const;

        /**
         * @brief High traction battery temperate warning flag.
         */
        bool warn_batt_temp_high() const;

        /**
         * @brief Low traction battery SOC warning flag.
         */
        bool warn_batt_soc_low() const;

        /**
         * @brief Check for open-circuit fault in the shutdown circuit / HVIL
        */
        bool fault_hvil_open() const;

        /**
         * @brief Check for short-circuit fault in the shutdown circuit / HVIL
        */
        bool fault_hvil_short() const;

        /**
         * @brief Check for Emergency Braking System fault
        */
        bool fault_ebs() const;

        /**
         * @brief Check for fault in the offboard charger
        */
        bool fault_offboard_charger() const;

        /**
         * @brief Check for loss of communication between AI and VCU
        */
        bool fault_ai_comms_lost() const;

        bool fault_autonomous_braking() const;
        bool fault_mission_status() const;
        bool fault_charge_procedure() const;
        bool fault_bms() const;
        bool fault_brake_plausibility() const;

    private:
        virtual void _process( const can_frame& frame ) override;
        virtual void _print( std::ostream& os ) const override;

        candata_vcu2_ai_status_t data;
    };
};

#endif // STATUS_FRAME_HPP
