#include <control/status_control_frame.hpp>

can_frame adas_api::StatusControlFrame::_process() const
{
    bool drive = this->_drive;
    bool estop = this->_estop;
    MissionStatus mission = this->_mission_status; // Confirm

    /* check that we have up to date information otherwise keep as safe values */
    if( this->_statusFrame.is_recent() )
    {
        // estop
        switch( this->_statusFrame.as_state() )
        {
            default:
                [[fallthrough]];
            case StatusFrame::AsState::EMERGENCY:
                [[fallthrough]];
            case StatusFrame::AsState::DRIVING:
                [[fallthrough]];
            case StatusFrame::AsState::FINISHED:
                break;

            case StatusFrame::AsState::OFF:
                [[fallthrough]];
            case StatusFrame::AsState::READY:
                estop = false;
                break;
        }
        
        // drive
        switch( this->_statusFrame.as_state() )
        {
            default:
                [[fallthrough]];
            case StatusFrame::AsState::OFF:
                [[fallthrough]];
            case StatusFrame::AsState::READY:
                [[fallthrough]];
            case StatusFrame::AsState::EMERGENCY:
                [[fallthrough]];
            case StatusFrame::AsState::FINISHED:
                drive = false;
                break;

            case StatusFrame::AsState::DRIVING:
                break;
        }
    }

    /* vehicle is DRIVING, can only change to FINISHED if vehicle is stopped */
    if( mission == MissionStatus::FINISHED && 
        this->_statusFrame.as_state() == StatusFrame::AsState::DRIVING &&
        ( !this->_frontFrame.is_recent() || !this->_rearFrame.is_recent() ||
          !this->_frontFrame.stopped() || !this->_rearFrame.stopped() ) )
    {
        mission = MissionStatus::RUNNING;
    }

    can_frame frame
        {
            .can_id = this->frameId,
            .can_dlc = CANDATA_AI2_VCU_STATUS_LENGTH
        };

    const uint8_t directionVal = drive ? CANDATA_AI2_VCU_STATUS_DIRECTION_REQUEST_FORWARD_CHOICE : 
                                         CANDATA_AI2_VCU_STATUS_DIRECTION_REQUEST_NEUTRAL_CHOICE;
    const uint8_t estopVal = estop ? CANDATA_AI2_VCU_STATUS_ESTOP_REQUEST_SHUTDOWN_REQUESTED_CHOICE :
                                     CANDATA_AI2_VCU_STATUS_ESTOP_REQUEST_NO_SHUTDOWN_CHOICE;
    const uint8_t handshakeVal = static_cast<uint8_t>( this->_statusFrame.handshake() );
    const uint8_t missionVal = static_cast<uint8_t>( mission );

    const candata_ai2_vcu_status_t data
        {
            .handshake = candata_ai2_vcu_status_handshake_encode( handshakeVal ),
            .estop_request = candata_ai2_vcu_status_estop_request_encode( estopVal ),
            .mission_status = candata_ai2_vcu_status_mission_status_encode( missionVal ),
            .direction_request = candata_ai2_vcu_status_direction_request_encode( directionVal ),
            /* The following signals are apparently part of FSG logging requirements but
                not apparently used by us at the moment.
            */
            .lap_counter = candata_ai2_vcu_status_lap_counter_encode( 0 ),
            .cones_count_actual = candata_ai2_vcu_status_cones_count_actual_encode( 0 ),
            .cones_count_all = candata_ai2_vcu_status_cones_count_all_encode( 0 ),
            .veh_speed_actual = candata_ai2_vcu_status_veh_speed_actual_encode( 0 ),
            .veh_speed_demand = candata_ai2_vcu_status_veh_speed_demand_encode( 0 )
        };
 
    /*if( !candata_ai2_vcu_steer_steer_request_is_in_range( data.steer_request ) )
    {
        throw std::runtime_error( "Steering angle out of range" );
    }*/

    candata_ai2_vcu_status_pack( frame.data, &data, frame.can_dlc );
 
    return frame;
}

void adas_api::StatusControlFrame::set_drive( const bool drive )
{
    this->_drive = drive;
}

void adas_api::StatusControlFrame::set_estop( const bool estop )
{
    this->_estop = estop;
}

void adas_api::StatusControlFrame::_print( std::ostream& os ) const
{
}

adas_api::StatusControlFrame::StatusControlFrame( const AxleTorqueFrame &frontFrame, 
        const AxleTorqueFrame &rearFrame, 
        const StatusFrame &statusFrame ) : 
    ControlFrame( CANDATA_AI2_VCU_STATUS_FRAME_ID ),
    _statusFrame( statusFrame ), 
    _frontFrame( frontFrame ), 
    _rearFrame( rearFrame ) {}