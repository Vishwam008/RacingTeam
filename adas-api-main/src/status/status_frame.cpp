#include "status/status_frame.hpp"

#include <sstream>

bool adas_api::StatusFrame::handshake() const
{
    return candata_vcu2_ai_status_handshake_decode( this->data.handshake );
}

bool adas_api::StatusFrame::StatusFrame::fault() const
{
    return fault_hvil_open() ||
        fault_hvil_short() ||
        fault_ebs() ||
        fault_offboard_charger() ||
        fault_ai_comms_lost() ||
        fault_autonomous_braking() ||
        fault_mission_status() ||
        fault_charge_procedure() ||
        fault_bms() ||
        fault_brake_plausibility();
}

const std::string adas_api::StatusFrame::fault_message() const
{
    std::stringstream ss;

    if( fault_hvil_open() )
        ss << "Open-circuit fault in the shutdown circuit / HVIL. ";
    if( fault_hvil_short() )
        ss << "Short-circuit fault in the shutdown circuit / HVIL. ";
    if( fault_ebs() )
        ss << "Emergency Braking System fault. ";
    if( fault_offboard_charger() )
        ss << "Fault in the offboard charger. ";
    if( fault_ai_comms_lost() )
        ss << "Loss of communication between AI and VCU. ";
    if( fault_autonomous_braking() )
        ss << "Neutral direction request made while vehicle is still moving. ";
    if( fault_mission_status() )
        ss << "Mission status set to finished while vehicle is moving. ";
    if( fault_charge_procedure() )
        ss << "AS or TS master switches are on when the battery is being charged. ";
    if( fault_bms() )
        ss << "Battery Management System fault. ";
    if( fault_brake_plausibility() )
        ss << "A non-zero drive torque demand is applied at the same time as a non-zero brake pressure demand. ";

    return ss.str();
}

const std::string adas_api::StatusFrame::warning_message() const
{
    std::stringstream ss;

    if( warning_status() )
        ss << "Warning. ";
    if( warn_batt_temp_high() )
        ss << "Battery temperature is too high. ";
    else if( warn_batt_soc_low() )
        ss << "Battery state of charge is too low. ";

    return ss.str();
}

bool adas_api::StatusFrame::warning() const
{
    return warning_status() || warn_batt_temp_high() || warn_batt_soc_low();
}

bool adas_api::StatusFrame::warning_status() const
{
    return candata_vcu2_ai_status_warning_status_decode( data.warning_status ) 
        == CANDATA_VCU2_AI_STATUS_WARNING_STATUS_WARNING_ACTIVE_CHOICE;
}

bool adas_api::StatusFrame::warn_batt_temp_high() const
{
    return candata_vcu2_ai_status_warn_batt_temp_high_decode( data.warn_batt_temp_high ) 
        == CANDATA_VCU2_AI_STATUS_WARN_BATT_TEMP_HIGH_TRUE_CHOICE;
}

bool adas_api::StatusFrame::warn_batt_soc_low() const
{
    return candata_vcu2_ai_status_warn_batt_soc_low_decode( data.warn_batt_soc_low ) 
        == CANDATA_VCU2_AI_STATUS_WARN_BATT_SOC_LOW_TRUE_CHOICE;
}

bool adas_api::StatusFrame::fault_hvil_open() const
{
    return candata_vcu2_ai_status_hvil_open_fault_decode( data.hvil_open_fault )
        == CANDATA_VCU2_AI_STATUS_HVIL_OPEN_FAULT_TRUE_CHOICE;
}

bool adas_api::StatusFrame::fault_hvil_short() const
{
    return candata_vcu2_ai_status_hvil_short_fault_decode( data.hvil_short_fault )
        == CANDATA_VCU2_AI_STATUS_HVIL_SHORT_FAULT_TRUE_CHOICE;
}

bool adas_api::StatusFrame::fault_ebs() const
{
    return candata_vcu2_ai_status_ebs_fault_decode( data.ebs_fault )
        == CANDATA_VCU2_AI_STATUS_EBS_FAULT_TRUE_CHOICE;
}

bool adas_api::StatusFrame::fault_offboard_charger() const
{
    return candata_vcu2_ai_status_offboard_charger_fault_decode( data.offboard_charger_fault ) 
        == CANDATA_VCU2_AI_STATUS_OFFBOARD_CHARGER_FAULT_TRUE_CHOICE;
}

bool adas_api::StatusFrame::fault_ai_comms_lost() const
{
    return candata_vcu2_ai_status_ai_comms_lost_decode( data.ai_comms_lost ) 
        == CANDATA_VCU2_AI_STATUS_AI_COMMS_LOST_TRUE_CHOICE;
}

bool adas_api::StatusFrame::fault_autonomous_braking() const
{
    return candata_vcu2_ai_status_autonomous_braking_fault_decode( data.autonomous_braking_fault ) 
        == CANDATA_VCU2_AI_STATUS_AUTONOMOUS_BRAKING_FAULT_TRUE_CHOICE;
}
bool adas_api::StatusFrame::fault_mission_status() const
{
    return candata_vcu2_ai_status_mission_status_fault_decode( data.mission_status_fault ) 
        == CANDATA_VCU2_AI_STATUS_MISSION_STATUS_FAULT_TRUE_CHOICE;
}
bool adas_api::StatusFrame::fault_charge_procedure() const
{
    return candata_vcu2_ai_status_charge_procedure_fault_decode( data.charge_procedure_fault ) 
        == CANDATA_VCU2_AI_STATUS_CHARGE_PROCEDURE_FAULT_TRUE_CHOICE;
}
bool adas_api::StatusFrame::fault_bms() const
{
    return candata_vcu2_ai_status_bms_fault_decode( data.bms_fault ) 
        == CANDATA_VCU2_AI_STATUS_BMS_FAULT_TRUE_CHOICE;
}
bool adas_api::StatusFrame::fault_brake_plausibility() const
{
    return candata_vcu2_ai_status_brake_plausibility_fault_decode( data.brake_plausibility_fault ) 
        == CANDATA_VCU2_AI_STATUS_BRAKE_PLAUSIBILITY_FAULT_TRUE_CHOICE;
}


void adas_api::StatusFrame::_process( const can_frame& frame )
{
    if( candata_vcu2_ai_status_unpack( &data, frame.data, frame.can_dlc ) )
    {
        throw std::runtime_error( "Failed to unpack status frame" );
    }
}

void adas_api::StatusFrame::_print( std::ostream& os ) const
{
    os << "Status: ";
    if( fault() )
        os << "Fault - " << fault_message() << ", ";
    if( warning() )
        os << "Warning - " << warning_message();
    /*os << "Fault: " << fault() << std::endl;
    os << "Fault message: " << fault_message() << std::endl;
    os << "Warning: " << warning_status() << std::endl;
    os << "Warning message: " << warning_message() << std::endl;*/
}

adas_api::StatusFrame::StatusFrame() : 
    ProcessFrame( CANDATA_VCU2_AI_STATUS_FRAME_ID )
{
}
