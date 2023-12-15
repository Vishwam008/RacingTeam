#ifndef STATUS_CONTROL_FRAME_HPP
#define STATUS_CONTROL_FRAME_HPP
#pragma once
#include "control/control_frame.hpp"
#include "status/status_frame.hpp"
#include "status/axle_torque_frame.hpp"

namespace adas_api{
    class StatusControlFrame : public ControlFrame
    {
    public:
        enum MissionStatus
        {
            NOT_SELECTED = CANDATA_AI2_VCU_STATUS_MISSION_STATUS_NOT_SELECTED_CHOICE,
            SELECTED = CANDATA_AI2_VCU_STATUS_MISSION_STATUS_SELECTED_CHOICE,
            RUNNING = CANDATA_AI2_VCU_STATUS_MISSION_STATUS_RUNNING_CHOICE,
            FINISHED = CANDATA_AI2_VCU_STATUS_MISSION_STATUS_FINISHED_CHOICE
        };

        StatusControlFrame( const AxleTorqueFrame &frontFrame, const AxleTorqueFrame &rearFrame, const StatusFrame &statusFrame );
        virtual ~StatusControlFrame() override = default;

        /**
         * @brief Set the drive object
         * 
         * @warning Although the CAN frame specifications say that the vehicle has
         *  a reverse mode, in reality this is disabled and so ignored by this API.
         * 
         * @param forward 
         */
        void set_drive( const bool forward );
        void set_estop( const bool estop );

        void set_mission_status( const MissionStatus status )
        {
            _mission_status = status;
        }

        MissionStatus get_mission_status() const
        {
            return this->_mission_status;
        }

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

        const StatusFrame& _statusFrame;
        const AxleTorqueFrame& _frontFrame;
        const AxleTorqueFrame& _rearFrame;

        bool _drive = false;
        bool _estop = false;
        MissionStatus _mission_status = MissionStatus::NOT_SELECTED;
    };
};

#endif // STATUS_CONTROL_FRAME_HPP
