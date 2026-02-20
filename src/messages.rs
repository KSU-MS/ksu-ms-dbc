// Generated code!
#![allow(unused_comparisons, unreachable_patterns)]
#![allow(clippy::let_and_return, clippy::eq_op)]
#![allow(clippy::excessive_precision, clippy::manual_range_contains, clippy::absurd_extreme_comparisons)]
#![deny(clippy::arithmetic_side_effects)]

//! Message definitions from file `"car.dbc"`
//!
//! - Version: `Version("")`

use core::ops::BitOr;
use bitvec::prelude::*;
#[cfg(feature = "arb")]
use arbitrary::{Arbitrary, Unstructured};

/// All messages
#[derive(Clone)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum Messages {
    /// module1_temps
    Module1Temps(Module1Temps),
    /// module2_temps
    Module2Temps(Module2Temps),
    /// module3_temps
    Module3Temps(Module3Temps),
    /// module4_temps
    Module4Temps(Module4Temps),
    /// module5_temps
    Module5Temps(Module5Temps),
    /// precharge_status
    PrechargeStatus(PrechargeStatus),
    /// module1_status
    Module1Status(Module1Status),
    /// module2_status
    Module2Status(Module2Status),
    /// module3_status
    Module3Status(Module3Status),
    /// module4_status
    Module4Status(Module4Status),
    /// module5_status
    Module5Status(Module5Status),
    /// VCU_STATUS
    VcuStatus(VcuStatus),
    /// VCU_PEDAL_READINGS
    VcuPedalReadings(VcuPedalReadings),
    /// VCU_WHEELSPEED_READINGS
    VcuWheelspeedReadings(VcuWheelspeedReadings),
    /// vcu_pedal_threshold_settings
    VcuPedalThresholdSettings(VcuPedalThresholdSettings),
    /// vcu_board_data
    VcuBoardData(VcuBoardData),
    /// vcu_board_readings_one
    VcuBoardReadingsOne(VcuBoardReadingsOne),
    /// vcu_board_readings_two
    VcuBoardReadingsTwo(VcuBoardReadingsTwo),
    /// vcu_pedals_travel
    VcuPedalsTravel(VcuPedalsTravel),
    /// vcu_launchcontrol_countdown
    VcuLaunchcontrolCountdown(VcuLaunchcontrolCountdown),
    /// vcu_distance_tracker_motor
    VcuDistanceTrackerMotor(VcuDistanceTrackerMotor),
    /// vcu_distance_tracker_wheelspeed
    VcuDistanceTrackerWheelspeed(VcuDistanceTrackerWheelspeed),
    /// vcu_lifetime_distance_and_ontime
    VcuLifetimeDistanceAndOntime(VcuLifetimeDistanceAndOntime),
    /// vcu_distance_tracker_vectornav
    VcuDistanceTrackerVectornav(VcuDistanceTrackerVectornav),
    /// vcu_coulomb_counters
    VcuCoulombCounters(VcuCoulombCounters),
    /// vcu_slip_info
    VcuSlipInfo(VcuSlipInfo),
    /// vcu_set_parameter
    VcuSetParameter(VcuSetParameter),
    /// dash_buttons
    DashButtons(DashButtons),
    /// dash_board_data
    DashBoardData(DashBoardData),
    /// evelogger_vectornav_attitude
    EveloggerVectornavAttitude(EveloggerVectornavAttitude),
    /// evelogger_vectornav_gyro
    EveloggerVectornavGyro(EveloggerVectornavGyro),
    /// evelogger_vectornav_position
    EveloggerVectornavPosition(EveloggerVectornavPosition),
    /// evelogger_vectornav_velocity
    EveloggerVectornavVelocity(EveloggerVectornavVelocity),
    /// evelogger_vectornav_acceleration
    EveloggerVectornavAcceleration(EveloggerVectornavAcceleration),
    /// evelogger_vectornav_time
    EveloggerVectornavTime(EveloggerVectornavTime),
    /// acu_shutdown_status
    AcuShutdownStatus(AcuShutdownStatus),
    /// acu_board_voltage_readings
    AcuBoardVoltageReadings(AcuBoardVoltageReadings),
    /// acu_board_data
    AcuBoardData(AcuBoardData),
    /// cornernode_steeringpot
    CornernodeSteeringpot(CornernodeSteeringpot),
    /// cornernode_fl_shockpot
    CornernodeFlShockpot(CornernodeFlShockpot),
    /// cornernode_fr_shockpot
    CornernodeFrShockpot(CornernodeFrShockpot),
    /// cornernode_rl_shockpot
    CornernodeRlShockpot(CornernodeRlShockpot),
    /// cornernode_rr_shockpot
    CornernodeRrShockpot(CornernodeRrShockpot),
    /// cornernode_fl_wheelspeed
    CornernodeFlWheelspeed(CornernodeFlWheelspeed),
    /// cornernode_fr_wheelspeed
    CornernodeFrWheelspeed(CornernodeFrWheelspeed),
    /// cornernode_rl_wheelspeed
    CornernodeRlWheelspeed(CornernodeRlWheelspeed),
    /// cornernode_rr_wheelspeed
    CornernodeRrWheelspeed(CornernodeRrWheelspeed),
    /// cornernode_fl_tiretemp
    CornernodeFlTiretemp(CornernodeFlTiretemp),
    /// cornernode_fr_tiretemp
    CornernodeFrTiretemp(CornernodeFrTiretemp),
    /// cornernode_rl_tiretemp
    CornernodeRlTiretemp(CornernodeRlTiretemp),
    /// cornernode_rr_tiretemp
    CornernodeRrTiretemp(CornernodeRrTiretemp),
    /// cornernode_front_brakepressure
    CornernodeFrontBrakepressure(CornernodeFrontBrakepressure),
    /// cornernode_rear_brakepressure
    CornernodeRearBrakepressure(CornernodeRearBrakepressure),
    /// cornernode_motor_temp
    CornernodeMotorTemp(CornernodeMotorTemp),
    /// cornernode_coolant_temp
    CornernodeCoolantTemp(CornernodeCoolantTemp),
    /// MSGID_0X6B1
    Msgid0x6b1(Msgid0x6b1),
    /// MSGID_0X6B2
    Msgid0x6b2(Msgid0x6b2),
    /// MSGID_0X6B3
    Msgid0x6b3(Msgid0x6b3),
    /// MSGID_0X6B4
    Msgid0x6b4(Msgid0x6b4),
    /// MSGID_0X1806E7F4
    Msgid0x1806e7f4(Msgid0x1806e7f4),
    /// MSGID_0X1806E5F4
    Msgid0x1806e5f4(Msgid0x1806e5f4),
    /// MSGID_0X1806E9F4
    Msgid0x1806e9f4(Msgid0x1806e9f4),
    /// M173_Modulation_And_Flux_Info
    M173ModulationAndFluxInfo(M173ModulationAndFluxInfo),
    /// M172_Torque_And_Timer_Info
    M172TorqueAndTimerInfo(M172TorqueAndTimerInfo),
    /// M194_Read_Write_Param_Response
    M194ReadWriteParamResponse(M194ReadWriteParamResponse),
    /// M193_Read_Write_Param_Command
    M193ReadWriteParamCommand(M193ReadWriteParamCommand),
    /// M192_Command_Message
    M192CommandMessage(M192CommandMessage),
    /// M171_Fault_Codes
    M171FaultCodes(M171FaultCodes),
    /// M170_Internal_States
    M170InternalStates(M170InternalStates),
    /// M169_Internal_Voltages
    M169InternalVoltages(M169InternalVoltages),
    /// M168_Flux_ID_IQ_Info
    M168FluxIdIqInfo(M168FluxIdIqInfo),
    /// M167_Voltage_Info
    M167VoltageInfo(M167VoltageInfo),
    /// M166_Current_Info
    M166CurrentInfo(M166CurrentInfo),
    /// M165_Motor_Position_Info
    M165MotorPositionInfo(M165MotorPositionInfo),
    /// M164_Digital_Input_Status
    M164DigitalInputStatus(M164DigitalInputStatus),
    /// M163_Analog_Input_Voltages
    M163AnalogInputVoltages(M163AnalogInputVoltages),
    /// M162_Temperature_Set_3
    M162TemperatureSet3(M162TemperatureSet3),
    /// M161_Temperature_Set_2
    M161TemperatureSet2(M161TemperatureSet2),
    /// M160_Temperature_Set_1
    M160TemperatureSet1(M160TemperatureSet1),
    /// M174_Firmware_Info
    M174FirmwareInfo(M174FirmwareInfo),
    /// M175_Diag_Data_Message
    M175DiagDataMessage(M175DiagDataMessage),
    /// BMS_Current_Limit
    BmsCurrentLimit(BmsCurrentLimit),
    /// M176_Fast_Info
    M176FastInfo(M176FastInfo),
    /// M177_Torque_Capability
    M177TorqueCapability(M177TorqueCapability),
    /// Status
    Status(Status),
}

impl Messages {
    /// Read message from CAN frame
    #[inline(never)]
    pub fn from_can_message(id: u32, payload: &[u8]) -> Result<Self, CanError> {
        
        let res = match id {
            31 => Messages::Module1Temps(Module1Temps::try_from(payload)?),
            32 => Messages::Module2Temps(Module2Temps::try_from(payload)?),
            33 => Messages::Module3Temps(Module3Temps::try_from(payload)?),
            34 => Messages::Module4Temps(Module4Temps::try_from(payload)?),
            35 => Messages::Module5Temps(Module5Temps::try_from(payload)?),
            105 => Messages::PrechargeStatus(PrechargeStatus::try_from(payload)?),
            131 => Messages::Module1Status(Module1Status::try_from(payload)?),
            132 => Messages::Module2Status(Module2Status::try_from(payload)?),
            133 => Messages::Module3Status(Module3Status::try_from(payload)?),
            134 => Messages::Module4Status(Module4Status::try_from(payload)?),
            135 => Messages::Module5Status(Module5Status::try_from(payload)?),
            195 => Messages::VcuStatus(VcuStatus::try_from(payload)?),
            196 => Messages::VcuPedalReadings(VcuPedalReadings::try_from(payload)?),
            198 => Messages::VcuWheelspeedReadings(VcuWheelspeedReadings::try_from(payload)?),
            199 => Messages::VcuPedalThresholdSettings(VcuPedalThresholdSettings::try_from(payload)?),
            200 => Messages::VcuBoardData(VcuBoardData::try_from(payload)?),
            201 => Messages::VcuBoardReadingsOne(VcuBoardReadingsOne::try_from(payload)?),
            202 => Messages::VcuBoardReadingsTwo(VcuBoardReadingsTwo::try_from(payload)?),
            204 => Messages::VcuPedalsTravel(VcuPedalsTravel::try_from(payload)?),
            205 => Messages::VcuLaunchcontrolCountdown(VcuLaunchcontrolCountdown::try_from(payload)?),
            206 => Messages::VcuDistanceTrackerMotor(VcuDistanceTrackerMotor::try_from(payload)?),
            207 => Messages::VcuDistanceTrackerWheelspeed(VcuDistanceTrackerWheelspeed::try_from(payload)?),
            208 => Messages::VcuLifetimeDistanceAndOntime(VcuLifetimeDistanceAndOntime::try_from(payload)?),
            210 => Messages::VcuDistanceTrackerVectornav(VcuDistanceTrackerVectornav::try_from(payload)?),
            211 => Messages::VcuCoulombCounters(VcuCoulombCounters::try_from(payload)?),
            212 => Messages::VcuSlipInfo(VcuSlipInfo::try_from(payload)?),
            214 => Messages::VcuSetParameter(VcuSetParameter::try_from(payload)?),
            235 => Messages::DashButtons(DashButtons::try_from(payload)?),
            236 => Messages::DashBoardData(DashBoardData::try_from(payload)?),
            500 => Messages::EveloggerVectornavAttitude(EveloggerVectornavAttitude::try_from(payload)?),
            501 => Messages::EveloggerVectornavGyro(EveloggerVectornavGyro::try_from(payload)?),
            502 => Messages::EveloggerVectornavPosition(EveloggerVectornavPosition::try_from(payload)?),
            503 => Messages::EveloggerVectornavVelocity(EveloggerVectornavVelocity::try_from(payload)?),
            504 => Messages::EveloggerVectornavAcceleration(EveloggerVectornavAcceleration::try_from(payload)?),
            505 => Messages::EveloggerVectornavTime(EveloggerVectornavTime::try_from(payload)?),
            600 => Messages::AcuShutdownStatus(AcuShutdownStatus::try_from(payload)?),
            601 => Messages::AcuBoardVoltageReadings(AcuBoardVoltageReadings::try_from(payload)?),
            602 => Messages::AcuBoardData(AcuBoardData::try_from(payload)?),
            899 => Messages::CornernodeSteeringpot(CornernodeSteeringpot::try_from(payload)?),
            900 => Messages::CornernodeFlShockpot(CornernodeFlShockpot::try_from(payload)?),
            901 => Messages::CornernodeFrShockpot(CornernodeFrShockpot::try_from(payload)?),
            902 => Messages::CornernodeRlShockpot(CornernodeRlShockpot::try_from(payload)?),
            903 => Messages::CornernodeRrShockpot(CornernodeRrShockpot::try_from(payload)?),
            904 => Messages::CornernodeFlWheelspeed(CornernodeFlWheelspeed::try_from(payload)?),
            905 => Messages::CornernodeFrWheelspeed(CornernodeFrWheelspeed::try_from(payload)?),
            906 => Messages::CornernodeRlWheelspeed(CornernodeRlWheelspeed::try_from(payload)?),
            907 => Messages::CornernodeRrWheelspeed(CornernodeRrWheelspeed::try_from(payload)?),
            908 => Messages::CornernodeFlTiretemp(CornernodeFlTiretemp::try_from(payload)?),
            909 => Messages::CornernodeFrTiretemp(CornernodeFrTiretemp::try_from(payload)?),
            910 => Messages::CornernodeRlTiretemp(CornernodeRlTiretemp::try_from(payload)?),
            911 => Messages::CornernodeRrTiretemp(CornernodeRrTiretemp::try_from(payload)?),
            912 => Messages::CornernodeFrontBrakepressure(CornernodeFrontBrakepressure::try_from(payload)?),
            913 => Messages::CornernodeRearBrakepressure(CornernodeRearBrakepressure::try_from(payload)?),
            914 => Messages::CornernodeMotorTemp(CornernodeMotorTemp::try_from(payload)?),
            915 => Messages::CornernodeCoolantTemp(CornernodeCoolantTemp::try_from(payload)?),
            1713 => Messages::Msgid0x6b1(Msgid0x6b1::try_from(payload)?),
            1714 => Messages::Msgid0x6b2(Msgid0x6b2::try_from(payload)?),
            1715 => Messages::Msgid0x6b3(Msgid0x6b3::try_from(payload)?),
            1716 => Messages::Msgid0x6b4(Msgid0x6b4::try_from(payload)?),
            2550589428 => Messages::Msgid0x1806e7f4(Msgid0x1806e7f4::try_from(payload)?),
            2550588916 => Messages::Msgid0x1806e5f4(Msgid0x1806e5f4::try_from(payload)?),
            2550589940 => Messages::Msgid0x1806e9f4(Msgid0x1806e9f4::try_from(payload)?),
            173 => Messages::M173ModulationAndFluxInfo(M173ModulationAndFluxInfo::try_from(payload)?),
            172 => Messages::M172TorqueAndTimerInfo(M172TorqueAndTimerInfo::try_from(payload)?),
            194 => Messages::M194ReadWriteParamResponse(M194ReadWriteParamResponse::try_from(payload)?),
            193 => Messages::M193ReadWriteParamCommand(M193ReadWriteParamCommand::try_from(payload)?),
            192 => Messages::M192CommandMessage(M192CommandMessage::try_from(payload)?),
            171 => Messages::M171FaultCodes(M171FaultCodes::try_from(payload)?),
            170 => Messages::M170InternalStates(M170InternalStates::try_from(payload)?),
            169 => Messages::M169InternalVoltages(M169InternalVoltages::try_from(payload)?),
            168 => Messages::M168FluxIdIqInfo(M168FluxIdIqInfo::try_from(payload)?),
            167 => Messages::M167VoltageInfo(M167VoltageInfo::try_from(payload)?),
            166 => Messages::M166CurrentInfo(M166CurrentInfo::try_from(payload)?),
            165 => Messages::M165MotorPositionInfo(M165MotorPositionInfo::try_from(payload)?),
            164 => Messages::M164DigitalInputStatus(M164DigitalInputStatus::try_from(payload)?),
            163 => Messages::M163AnalogInputVoltages(M163AnalogInputVoltages::try_from(payload)?),
            162 => Messages::M162TemperatureSet3(M162TemperatureSet3::try_from(payload)?),
            161 => Messages::M161TemperatureSet2(M161TemperatureSet2::try_from(payload)?),
            160 => Messages::M160TemperatureSet1(M160TemperatureSet1::try_from(payload)?),
            174 => Messages::M174FirmwareInfo(M174FirmwareInfo::try_from(payload)?),
            175 => Messages::M175DiagDataMessage(M175DiagDataMessage::try_from(payload)?),
            514 => Messages::BmsCurrentLimit(BmsCurrentLimit::try_from(payload)?),
            176 => Messages::M176FastInfo(M176FastInfo::try_from(payload)?),
            177 => Messages::M177TorqueCapability(M177TorqueCapability::try_from(payload)?),
            2566869221 => Messages::Status(Status::try_from(payload)?),
            n => return Err(CanError::UnknownMessageId(n)),
        };
        Ok(res)
    }
}

/// module1_temps
///
/// - ID: 31 (0x1f)
/// - Size: 8 bytes
/// - Transmitter: module1
///
/// raw ADC voltage reading of module1 temp sensors, relative to the car
#[derive(Clone, Copy)]
pub struct Module1Temps {
    raw: [u8; 8],
}

impl Module1Temps {
    pub const MESSAGE_ID: u32 = 31;
    
    pub const MODULE_VOLTAGE_8_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_8_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_7_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_7_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_6_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_6_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_5_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_5_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_4_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_4_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_3_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_3_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_2_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_2_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_1_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_1_MAX: f32 = 255_f32;
    
    /// Construct new module1_temps from values
    pub fn new(module_voltage_8: f32, module_voltage_7: f32, module_voltage_6: f32, module_voltage_5: f32, module_voltage_4: f32, module_voltage_3: f32, module_voltage_2: f32, module_voltage_1: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_module_voltage_8(module_voltage_8)?;
        res.set_module_voltage_7(module_voltage_7)?;
        res.set_module_voltage_6(module_voltage_6)?;
        res.set_module_voltage_5(module_voltage_5)?;
        res.set_module_voltage_4(module_voltage_4)?;
        res.set_module_voltage_3(module_voltage_3)?;
        res.set_module_voltage_2(module_voltage_2)?;
        res.set_module_voltage_1(module_voltage_1)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// module_voltage_8
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_8(&self) -> f32 {
        self.module_voltage_8_raw()
    }
    
    /// Get raw value of module_voltage_8
    ///
    /// - Start bit: 56
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_8_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[56..64].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_8
    #[inline(always)]
    pub fn set_module_voltage_8(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 31 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[56..64].store_le(value);
        Ok(())
    }
    
    /// module_voltage_7
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_7(&self) -> f32 {
        self.module_voltage_7_raw()
    }
    
    /// Get raw value of module_voltage_7
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_7_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_7
    #[inline(always)]
    pub fn set_module_voltage_7(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 31 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
    /// module_voltage_6
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_6(&self) -> f32 {
        self.module_voltage_6_raw()
    }
    
    /// Get raw value of module_voltage_6
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_6_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_6
    #[inline(always)]
    pub fn set_module_voltage_6(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 31 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// module_voltage_5
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_5(&self) -> f32 {
        self.module_voltage_5_raw()
    }
    
    /// Get raw value of module_voltage_5
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_5_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_5
    #[inline(always)]
    pub fn set_module_voltage_5(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 31 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// module_voltage_4
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_4(&self) -> f32 {
        self.module_voltage_4_raw()
    }
    
    /// Get raw value of module_voltage_4
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_4_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_4
    #[inline(always)]
    pub fn set_module_voltage_4(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 31 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// module_voltage_3
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_3(&self) -> f32 {
        self.module_voltage_3_raw()
    }
    
    /// Get raw value of module_voltage_3
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_3_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_3
    #[inline(always)]
    pub fn set_module_voltage_3(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 31 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// module_voltage_2
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_2(&self) -> f32 {
        self.module_voltage_2_raw()
    }
    
    /// Get raw value of module_voltage_2
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_2_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_2
    #[inline(always)]
    pub fn set_module_voltage_2(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 31 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// module_voltage_1
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_1(&self) -> f32 {
        self.module_voltage_1_raw()
    }
    
    /// Get raw value of module_voltage_1
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_1_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_1
    #[inline(always)]
    pub fn set_module_voltage_1(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 31 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module1Temps {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module1Temps {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module1Temps")
                .field("module_voltage_8", &self.module_voltage_8())
                .field("module_voltage_7", &self.module_voltage_7())
                .field("module_voltage_6", &self.module_voltage_6())
                .field("module_voltage_5", &self.module_voltage_5())
                .field("module_voltage_4", &self.module_voltage_4())
                .field("module_voltage_3", &self.module_voltage_3())
                .field("module_voltage_2", &self.module_voltage_2())
                .field("module_voltage_1", &self.module_voltage_1())
            .finish()
        } else {
            f.debug_tuple("Module1Temps").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module1Temps {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let module_voltage_8 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_7 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_6 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_5 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_4 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_3 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_2 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_1 = u.float_in_range(0_f32..=255_f32)?;
        Module1Temps::new(module_voltage_8,module_voltage_7,module_voltage_6,module_voltage_5,module_voltage_4,module_voltage_3,module_voltage_2,module_voltage_1).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module2_temps
///
/// - ID: 32 (0x20)
/// - Size: 8 bytes
/// - Transmitter: module2
///
/// raw ADC voltage reading of module2 temp sensors, relative to the car
#[derive(Clone, Copy)]
pub struct Module2Temps {
    raw: [u8; 8],
}

impl Module2Temps {
    pub const MESSAGE_ID: u32 = 32;
    
    pub const MODULE_VOLTAGE_8_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_8_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_7_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_7_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_6_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_6_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_5_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_5_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_4_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_4_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_3_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_3_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_2_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_2_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_1_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_1_MAX: f32 = 255_f32;
    
    /// Construct new module2_temps from values
    pub fn new(module_voltage_8: f32, module_voltage_7: f32, module_voltage_6: f32, module_voltage_5: f32, module_voltage_4: f32, module_voltage_3: f32, module_voltage_2: f32, module_voltage_1: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_module_voltage_8(module_voltage_8)?;
        res.set_module_voltage_7(module_voltage_7)?;
        res.set_module_voltage_6(module_voltage_6)?;
        res.set_module_voltage_5(module_voltage_5)?;
        res.set_module_voltage_4(module_voltage_4)?;
        res.set_module_voltage_3(module_voltage_3)?;
        res.set_module_voltage_2(module_voltage_2)?;
        res.set_module_voltage_1(module_voltage_1)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// module_voltage_8
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_8(&self) -> f32 {
        self.module_voltage_8_raw()
    }
    
    /// Get raw value of module_voltage_8
    ///
    /// - Start bit: 56
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_8_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[56..64].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_8
    #[inline(always)]
    pub fn set_module_voltage_8(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 32 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[56..64].store_le(value);
        Ok(())
    }
    
    /// module_voltage_7
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_7(&self) -> f32 {
        self.module_voltage_7_raw()
    }
    
    /// Get raw value of module_voltage_7
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_7_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_7
    #[inline(always)]
    pub fn set_module_voltage_7(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 32 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
    /// module_voltage_6
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_6(&self) -> f32 {
        self.module_voltage_6_raw()
    }
    
    /// Get raw value of module_voltage_6
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_6_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_6
    #[inline(always)]
    pub fn set_module_voltage_6(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 32 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// module_voltage_5
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_5(&self) -> f32 {
        self.module_voltage_5_raw()
    }
    
    /// Get raw value of module_voltage_5
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_5_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_5
    #[inline(always)]
    pub fn set_module_voltage_5(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 32 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// module_voltage_4
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_4(&self) -> f32 {
        self.module_voltage_4_raw()
    }
    
    /// Get raw value of module_voltage_4
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_4_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_4
    #[inline(always)]
    pub fn set_module_voltage_4(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 32 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// module_voltage_3
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_3(&self) -> f32 {
        self.module_voltage_3_raw()
    }
    
    /// Get raw value of module_voltage_3
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_3_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_3
    #[inline(always)]
    pub fn set_module_voltage_3(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 32 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// module_voltage_2
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_2(&self) -> f32 {
        self.module_voltage_2_raw()
    }
    
    /// Get raw value of module_voltage_2
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_2_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_2
    #[inline(always)]
    pub fn set_module_voltage_2(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 32 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// module_voltage_1
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_1(&self) -> f32 {
        self.module_voltage_1_raw()
    }
    
    /// Get raw value of module_voltage_1
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_1_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_1
    #[inline(always)]
    pub fn set_module_voltage_1(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 32 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module2Temps {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module2Temps {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module2Temps")
                .field("module_voltage_8", &self.module_voltage_8())
                .field("module_voltage_7", &self.module_voltage_7())
                .field("module_voltage_6", &self.module_voltage_6())
                .field("module_voltage_5", &self.module_voltage_5())
                .field("module_voltage_4", &self.module_voltage_4())
                .field("module_voltage_3", &self.module_voltage_3())
                .field("module_voltage_2", &self.module_voltage_2())
                .field("module_voltage_1", &self.module_voltage_1())
            .finish()
        } else {
            f.debug_tuple("Module2Temps").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module2Temps {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let module_voltage_8 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_7 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_6 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_5 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_4 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_3 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_2 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_1 = u.float_in_range(0_f32..=255_f32)?;
        Module2Temps::new(module_voltage_8,module_voltage_7,module_voltage_6,module_voltage_5,module_voltage_4,module_voltage_3,module_voltage_2,module_voltage_1).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module3_temps
///
/// - ID: 33 (0x21)
/// - Size: 8 bytes
/// - Transmitter: module3
///
/// raw ADC voltage reading of module3 temp sensors, relative to the car
#[derive(Clone, Copy)]
pub struct Module3Temps {
    raw: [u8; 8],
}

impl Module3Temps {
    pub const MESSAGE_ID: u32 = 33;
    
    pub const MODULE_VOLTAGE_8_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_8_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_7_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_7_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_6_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_6_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_5_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_5_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_4_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_4_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_3_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_3_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_2_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_2_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_1_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_1_MAX: f32 = 255_f32;
    
    /// Construct new module3_temps from values
    pub fn new(module_voltage_8: f32, module_voltage_7: f32, module_voltage_6: f32, module_voltage_5: f32, module_voltage_4: f32, module_voltage_3: f32, module_voltage_2: f32, module_voltage_1: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_module_voltage_8(module_voltage_8)?;
        res.set_module_voltage_7(module_voltage_7)?;
        res.set_module_voltage_6(module_voltage_6)?;
        res.set_module_voltage_5(module_voltage_5)?;
        res.set_module_voltage_4(module_voltage_4)?;
        res.set_module_voltage_3(module_voltage_3)?;
        res.set_module_voltage_2(module_voltage_2)?;
        res.set_module_voltage_1(module_voltage_1)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// module_voltage_8
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_8(&self) -> f32 {
        self.module_voltage_8_raw()
    }
    
    /// Get raw value of module_voltage_8
    ///
    /// - Start bit: 56
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_8_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[56..64].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_8
    #[inline(always)]
    pub fn set_module_voltage_8(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 33 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[56..64].store_le(value);
        Ok(())
    }
    
    /// module_voltage_7
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_7(&self) -> f32 {
        self.module_voltage_7_raw()
    }
    
    /// Get raw value of module_voltage_7
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_7_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_7
    #[inline(always)]
    pub fn set_module_voltage_7(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 33 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
    /// module_voltage_6
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_6(&self) -> f32 {
        self.module_voltage_6_raw()
    }
    
    /// Get raw value of module_voltage_6
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_6_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_6
    #[inline(always)]
    pub fn set_module_voltage_6(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 33 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// module_voltage_5
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_5(&self) -> f32 {
        self.module_voltage_5_raw()
    }
    
    /// Get raw value of module_voltage_5
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_5_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_5
    #[inline(always)]
    pub fn set_module_voltage_5(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 33 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// module_voltage_4
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_4(&self) -> f32 {
        self.module_voltage_4_raw()
    }
    
    /// Get raw value of module_voltage_4
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_4_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_4
    #[inline(always)]
    pub fn set_module_voltage_4(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 33 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// module_voltage_3
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_3(&self) -> f32 {
        self.module_voltage_3_raw()
    }
    
    /// Get raw value of module_voltage_3
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_3_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_3
    #[inline(always)]
    pub fn set_module_voltage_3(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 33 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// module_voltage_2
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_2(&self) -> f32 {
        self.module_voltage_2_raw()
    }
    
    /// Get raw value of module_voltage_2
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_2_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_2
    #[inline(always)]
    pub fn set_module_voltage_2(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 33 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// module_voltage_1
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_1(&self) -> f32 {
        self.module_voltage_1_raw()
    }
    
    /// Get raw value of module_voltage_1
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_1_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_1
    #[inline(always)]
    pub fn set_module_voltage_1(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 33 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module3Temps {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module3Temps {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module3Temps")
                .field("module_voltage_8", &self.module_voltage_8())
                .field("module_voltage_7", &self.module_voltage_7())
                .field("module_voltage_6", &self.module_voltage_6())
                .field("module_voltage_5", &self.module_voltage_5())
                .field("module_voltage_4", &self.module_voltage_4())
                .field("module_voltage_3", &self.module_voltage_3())
                .field("module_voltage_2", &self.module_voltage_2())
                .field("module_voltage_1", &self.module_voltage_1())
            .finish()
        } else {
            f.debug_tuple("Module3Temps").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module3Temps {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let module_voltage_8 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_7 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_6 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_5 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_4 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_3 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_2 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_1 = u.float_in_range(0_f32..=255_f32)?;
        Module3Temps::new(module_voltage_8,module_voltage_7,module_voltage_6,module_voltage_5,module_voltage_4,module_voltage_3,module_voltage_2,module_voltage_1).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module4_temps
///
/// - ID: 34 (0x22)
/// - Size: 8 bytes
/// - Transmitter: module4
///
/// raw ADC voltage reading of module4 temp sensors, relative to the car
#[derive(Clone, Copy)]
pub struct Module4Temps {
    raw: [u8; 8],
}

impl Module4Temps {
    pub const MESSAGE_ID: u32 = 34;
    
    pub const MODULE_VOLTAGE_8_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_8_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_7_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_7_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_6_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_6_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_5_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_5_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_4_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_4_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_3_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_3_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_2_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_2_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_1_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_1_MAX: f32 = 255_f32;
    
    /// Construct new module4_temps from values
    pub fn new(module_voltage_8: f32, module_voltage_7: f32, module_voltage_6: f32, module_voltage_5: f32, module_voltage_4: f32, module_voltage_3: f32, module_voltage_2: f32, module_voltage_1: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_module_voltage_8(module_voltage_8)?;
        res.set_module_voltage_7(module_voltage_7)?;
        res.set_module_voltage_6(module_voltage_6)?;
        res.set_module_voltage_5(module_voltage_5)?;
        res.set_module_voltage_4(module_voltage_4)?;
        res.set_module_voltage_3(module_voltage_3)?;
        res.set_module_voltage_2(module_voltage_2)?;
        res.set_module_voltage_1(module_voltage_1)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// module_voltage_8
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_8(&self) -> f32 {
        self.module_voltage_8_raw()
    }
    
    /// Get raw value of module_voltage_8
    ///
    /// - Start bit: 56
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_8_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[56..64].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_8
    #[inline(always)]
    pub fn set_module_voltage_8(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 34 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[56..64].store_le(value);
        Ok(())
    }
    
    /// module_voltage_7
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_7(&self) -> f32 {
        self.module_voltage_7_raw()
    }
    
    /// Get raw value of module_voltage_7
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_7_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_7
    #[inline(always)]
    pub fn set_module_voltage_7(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 34 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
    /// module_voltage_6
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_6(&self) -> f32 {
        self.module_voltage_6_raw()
    }
    
    /// Get raw value of module_voltage_6
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_6_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_6
    #[inline(always)]
    pub fn set_module_voltage_6(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 34 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// module_voltage_5
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_5(&self) -> f32 {
        self.module_voltage_5_raw()
    }
    
    /// Get raw value of module_voltage_5
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_5_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_5
    #[inline(always)]
    pub fn set_module_voltage_5(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 34 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// module_voltage_4
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_4(&self) -> f32 {
        self.module_voltage_4_raw()
    }
    
    /// Get raw value of module_voltage_4
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_4_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_4
    #[inline(always)]
    pub fn set_module_voltage_4(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 34 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// module_voltage_3
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_3(&self) -> f32 {
        self.module_voltage_3_raw()
    }
    
    /// Get raw value of module_voltage_3
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_3_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_3
    #[inline(always)]
    pub fn set_module_voltage_3(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 34 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// module_voltage_2
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_2(&self) -> f32 {
        self.module_voltage_2_raw()
    }
    
    /// Get raw value of module_voltage_2
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_2_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_2
    #[inline(always)]
    pub fn set_module_voltage_2(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 34 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// module_voltage_1
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_1(&self) -> f32 {
        self.module_voltage_1_raw()
    }
    
    /// Get raw value of module_voltage_1
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_1_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_1
    #[inline(always)]
    pub fn set_module_voltage_1(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 34 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module4Temps {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module4Temps {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module4Temps")
                .field("module_voltage_8", &self.module_voltage_8())
                .field("module_voltage_7", &self.module_voltage_7())
                .field("module_voltage_6", &self.module_voltage_6())
                .field("module_voltage_5", &self.module_voltage_5())
                .field("module_voltage_4", &self.module_voltage_4())
                .field("module_voltage_3", &self.module_voltage_3())
                .field("module_voltage_2", &self.module_voltage_2())
                .field("module_voltage_1", &self.module_voltage_1())
            .finish()
        } else {
            f.debug_tuple("Module4Temps").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module4Temps {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let module_voltage_8 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_7 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_6 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_5 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_4 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_3 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_2 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_1 = u.float_in_range(0_f32..=255_f32)?;
        Module4Temps::new(module_voltage_8,module_voltage_7,module_voltage_6,module_voltage_5,module_voltage_4,module_voltage_3,module_voltage_2,module_voltage_1).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module5_temps
///
/// - ID: 35 (0x23)
/// - Size: 8 bytes
/// - Transmitter: module5
///
/// raw ADC voltage reading of module5 temp sensors, relative to the car
#[derive(Clone, Copy)]
pub struct Module5Temps {
    raw: [u8; 8],
}

impl Module5Temps {
    pub const MESSAGE_ID: u32 = 35;
    
    pub const MODULE_VOLTAGE_8_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_8_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_7_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_7_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_6_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_6_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_5_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_5_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_4_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_4_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_3_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_3_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_2_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_2_MAX: f32 = 255_f32;
    pub const MODULE_VOLTAGE_1_MIN: f32 = 0_f32;
    pub const MODULE_VOLTAGE_1_MAX: f32 = 255_f32;
    
    /// Construct new module5_temps from values
    pub fn new(module_voltage_8: f32, module_voltage_7: f32, module_voltage_6: f32, module_voltage_5: f32, module_voltage_4: f32, module_voltage_3: f32, module_voltage_2: f32, module_voltage_1: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_module_voltage_8(module_voltage_8)?;
        res.set_module_voltage_7(module_voltage_7)?;
        res.set_module_voltage_6(module_voltage_6)?;
        res.set_module_voltage_5(module_voltage_5)?;
        res.set_module_voltage_4(module_voltage_4)?;
        res.set_module_voltage_3(module_voltage_3)?;
        res.set_module_voltage_2(module_voltage_2)?;
        res.set_module_voltage_1(module_voltage_1)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// module_voltage_8
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_8(&self) -> f32 {
        self.module_voltage_8_raw()
    }
    
    /// Get raw value of module_voltage_8
    ///
    /// - Start bit: 56
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_8_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[56..64].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_8
    #[inline(always)]
    pub fn set_module_voltage_8(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 35 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[56..64].store_le(value);
        Ok(())
    }
    
    /// module_voltage_7
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_7(&self) -> f32 {
        self.module_voltage_7_raw()
    }
    
    /// Get raw value of module_voltage_7
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_7_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_7
    #[inline(always)]
    pub fn set_module_voltage_7(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 35 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
    /// module_voltage_6
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_6(&self) -> f32 {
        self.module_voltage_6_raw()
    }
    
    /// Get raw value of module_voltage_6
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_6_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_6
    #[inline(always)]
    pub fn set_module_voltage_6(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 35 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// module_voltage_5
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_5(&self) -> f32 {
        self.module_voltage_5_raw()
    }
    
    /// Get raw value of module_voltage_5
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_5_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_5
    #[inline(always)]
    pub fn set_module_voltage_5(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 35 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// module_voltage_4
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_4(&self) -> f32 {
        self.module_voltage_4_raw()
    }
    
    /// Get raw value of module_voltage_4
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_4_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_4
    #[inline(always)]
    pub fn set_module_voltage_4(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 35 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// module_voltage_3
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_3(&self) -> f32 {
        self.module_voltage_3_raw()
    }
    
    /// Get raw value of module_voltage_3
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_3_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_3
    #[inline(always)]
    pub fn set_module_voltage_3(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 35 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// module_voltage_2
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_2(&self) -> f32 {
        self.module_voltage_2_raw()
    }
    
    /// Get raw value of module_voltage_2
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_2_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_2
    #[inline(always)]
    pub fn set_module_voltage_2(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 35 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// module_voltage_1
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn module_voltage_1(&self) -> f32 {
        self.module_voltage_1_raw()
    }
    
    /// Get raw value of module_voltage_1
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn module_voltage_1_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of module_voltage_1
    #[inline(always)]
    pub fn set_module_voltage_1(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 35 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module5Temps {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module5Temps {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module5Temps")
                .field("module_voltage_8", &self.module_voltage_8())
                .field("module_voltage_7", &self.module_voltage_7())
                .field("module_voltage_6", &self.module_voltage_6())
                .field("module_voltage_5", &self.module_voltage_5())
                .field("module_voltage_4", &self.module_voltage_4())
                .field("module_voltage_3", &self.module_voltage_3())
                .field("module_voltage_2", &self.module_voltage_2())
                .field("module_voltage_1", &self.module_voltage_1())
            .finish()
        } else {
            f.debug_tuple("Module5Temps").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module5Temps {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let module_voltage_8 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_7 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_6 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_5 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_4 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_3 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_2 = u.float_in_range(0_f32..=255_f32)?;
        let module_voltage_1 = u.float_in_range(0_f32..=255_f32)?;
        Module5Temps::new(module_voltage_8,module_voltage_7,module_voltage_6,module_voltage_5,module_voltage_4,module_voltage_3,module_voltage_2,module_voltage_1).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// precharge_status
///
/// - ID: 105 (0x69)
/// - Size: 8 bytes
#[derive(Clone, Copy)]
pub struct PrechargeStatus {
    raw: [u8; 8],
}

impl PrechargeStatus {
    pub const MESSAGE_ID: u32 = 105;
    
    pub const PRECHARGE_ERROR_CODE_MIN: u8 = 0_u8;
    pub const PRECHARGE_ERROR_CODE_MAX: u8 = 255_u8;
    pub const PRECHARGE_TS_VOLTAGE_DIV100_MIN: u8 = 0_u8;
    pub const PRECHARGE_TS_VOLTAGE_DIV100_MAX: u8 = 255_u8;
    pub const PRECHARGE_TS_VOLTAGE_MOD100_MIN: u8 = 0_u8;
    pub const PRECHARGE_TS_VOLTAGE_MOD100_MAX: u8 = 255_u8;
    pub const PRECHARGE_ACC_VOLTAGE_DIV100_MIN: u8 = 0_u8;
    pub const PRECHARGE_ACC_VOLTAGE_DIV100_MAX: u8 = 255_u8;
    pub const PRECHARGE_ACC_VOLTAGE_MOD100_MIN: u8 = 0_u8;
    pub const PRECHARGE_ACC_VOLTAGE_MOD100_MAX: u8 = 255_u8;
    pub const PRECHARGE_STATE_MIN: u8 = 0_u8;
    pub const PRECHARGE_STATE_MAX: u8 = 255_u8;
    
    /// Construct new precharge_status from values
    pub fn new(precharge_error_code: u8, precharge_ts_voltage_div100: u8, precharge_ts_voltage_mod100: u8, precharge_acc_voltage_div100: u8, precharge_acc_voltage_mod100: u8, precharge_state: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_precharge_error_code(precharge_error_code)?;
        res.set_precharge_ts_voltage_div100(precharge_ts_voltage_div100)?;
        res.set_precharge_ts_voltage_mod100(precharge_ts_voltage_mod100)?;
        res.set_precharge_acc_voltage_div100(precharge_acc_voltage_div100)?;
        res.set_precharge_acc_voltage_mod100(precharge_acc_voltage_mod100)?;
        res.set_precharge_state(precharge_state)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// precharge_errorCode
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn precharge_error_code(&self) -> u8 {
        self.precharge_error_code_raw()
    }
    
    /// Get raw value of precharge_errorCode
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn precharge_error_code_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        signal
    }
    
    /// Set value of precharge_errorCode
    #[inline(always)]
    pub fn set_precharge_error_code(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 105 });
        }
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// precharge_tsVoltageDiv100
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn precharge_ts_voltage_div100(&self) -> u8 {
        self.precharge_ts_voltage_div100_raw()
    }
    
    /// Get raw value of precharge_tsVoltageDiv100
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn precharge_ts_voltage_div100_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        signal
    }
    
    /// Set value of precharge_tsVoltageDiv100
    #[inline(always)]
    pub fn set_precharge_ts_voltage_div100(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 105 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// precharge_tsVoltageMod100
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn precharge_ts_voltage_mod100(&self) -> u8 {
        self.precharge_ts_voltage_mod100_raw()
    }
    
    /// Get raw value of precharge_tsVoltageMod100
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn precharge_ts_voltage_mod100_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        signal
    }
    
    /// Set value of precharge_tsVoltageMod100
    #[inline(always)]
    pub fn set_precharge_ts_voltage_mod100(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 105 });
        }
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// precharge_accVoltageDiv100
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn precharge_acc_voltage_div100(&self) -> u8 {
        self.precharge_acc_voltage_div100_raw()
    }
    
    /// Get raw value of precharge_accVoltageDiv100
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn precharge_acc_voltage_div100_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        signal
    }
    
    /// Set value of precharge_accVoltageDiv100
    #[inline(always)]
    pub fn set_precharge_acc_voltage_div100(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 105 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// precharge_accVoltageMod100
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn precharge_acc_voltage_mod100(&self) -> u8 {
        self.precharge_acc_voltage_mod100_raw()
    }
    
    /// Get raw value of precharge_accVoltageMod100
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn precharge_acc_voltage_mod100_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        signal
    }
    
    /// Set value of precharge_accVoltageMod100
    #[inline(always)]
    pub fn set_precharge_acc_voltage_mod100(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 105 });
        }
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// precharge_state
    ///
    /// 0 - standby, 1 - precharging, 2 - running, 3 - error
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn precharge_state(&self) -> u8 {
        self.precharge_state_raw()
    }
    
    /// Get raw value of precharge_state
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn precharge_state_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of precharge_state
    #[inline(always)]
    pub fn set_precharge_state(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 105 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for PrechargeStatus {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for PrechargeStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("PrechargeStatus")
                .field("precharge_error_code", &self.precharge_error_code())
                .field("precharge_ts_voltage_div100", &self.precharge_ts_voltage_div100())
                .field("precharge_ts_voltage_mod100", &self.precharge_ts_voltage_mod100())
                .field("precharge_acc_voltage_div100", &self.precharge_acc_voltage_div100())
                .field("precharge_acc_voltage_mod100", &self.precharge_acc_voltage_mod100())
                .field("precharge_state", &self.precharge_state())
            .finish()
        } else {
            f.debug_tuple("PrechargeStatus").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for PrechargeStatus {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let precharge_error_code = u.int_in_range(0..=255)?;
        let precharge_ts_voltage_div100 = u.int_in_range(0..=255)?;
        let precharge_ts_voltage_mod100 = u.int_in_range(0..=255)?;
        let precharge_acc_voltage_div100 = u.int_in_range(0..=255)?;
        let precharge_acc_voltage_mod100 = u.int_in_range(0..=255)?;
        let precharge_state = u.int_in_range(0..=255)?;
        PrechargeStatus::new(precharge_error_code,precharge_ts_voltage_div100,precharge_ts_voltage_mod100,precharge_acc_voltage_div100,precharge_acc_voltage_mod100,precharge_state).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module1_status
///
/// - ID: 131 (0x83)
/// - Size: 8 bytes
/// - Transmitter: module1
///
/// Git info for the firmware, on timer, along with temperature and humidity
#[derive(Clone, Copy)]
pub struct Module1Status {
    raw: [u8; 8],
}

impl Module1Status {
    pub const MESSAGE_ID: u32 = 131;
    
    pub const BOARD_HUMIDITY_MIN: u8 = 0_u8;
    pub const BOARD_HUMIDITY_MAX: u8 = 127_u8;
    pub const BOARD_TEMPERATURE_MIN: u8 = 0_u8;
    pub const BOARD_TEMPERATURE_MAX: u8 = 127_u8;
    pub const BOARD_ON_TIME_SECONDS_MIN: u16 = 0_u16;
    pub const BOARD_ON_TIME_SECONDS_MAX: u16 = 65535_u16;
    pub const FIRMWARE_VERSION_MIN: u32 = 0_u32;
    pub const FIRMWARE_VERSION_MAX: u32 = 4294967295_u32;
    
    /// Construct new module1_status from values
    pub fn new(board_humidity: u8, board_temperature: u8, firmware_is_dirty: bool, firmware_on_main: bool, board_on_time_seconds: u16, firmware_version: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_board_humidity(board_humidity)?;
        res.set_board_temperature(board_temperature)?;
        res.set_firmware_is_dirty(firmware_is_dirty)?;
        res.set_firmware_on_main(firmware_on_main)?;
        res.set_board_on_time_seconds(board_on_time_seconds)?;
        res.set_firmware_version(firmware_version)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// board_humidity
    ///
    /// the relative humidity at the acu
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "relative_humidity"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_humidity(&self) -> u8 {
        self.board_humidity_raw()
    }
    
    /// Get raw value of board_humidity
    ///
    /// - Start bit: 57
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_humidity_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[57..64].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_humidity
    #[inline(always)]
    pub fn set_board_humidity(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 131 });
        }
        self.raw.view_bits_mut::<Lsb0>()[57..64].store_le(value);
        Ok(())
    }
    
    /// board_temperature
    ///
    /// The ambient temperature at the PCB
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "celcius"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_temperature(&self) -> u8 {
        self.board_temperature_raw()
    }
    
    /// Get raw value of board_temperature
    ///
    /// - Start bit: 50
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[50..57].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_temperature
    #[inline(always)]
    pub fn set_board_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 131 });
        }
        self.raw.view_bits_mut::<Lsb0>()[50..57].store_le(value);
        Ok(())
    }
    
    /// firmware_is_dirty
    ///
    /// if the compiled firmware had uncommitted changes (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_is_dirty(&self) -> bool {
        self.firmware_is_dirty_raw()
    }
    
    /// Get raw value of firmware_is_dirty
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_is_dirty_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_is_dirty
    #[inline(always)]
    pub fn set_firmware_is_dirty(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// firmware_on_main
    ///
    /// if the compiled firmware was from main branch (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_on_main(&self) -> bool {
        self.firmware_on_main_raw()
    }
    
    /// Get raw value of firmware_on_main
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_on_main_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_on_main
    #[inline(always)]
    pub fn set_firmware_on_main(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// board_on_time_seconds
    ///
    /// time in seconds which the board has been powered on and running
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_on_time_seconds(&self) -> u16 {
        self.board_on_time_seconds_raw()
    }
    
    /// Get raw value of board_on_time_seconds
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_on_time_seconds_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of board_on_time_seconds
    #[inline(always)]
    pub fn set_board_on_time_seconds(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 131 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// firmware_version
    ///
    /// short hash of the firmware
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version_raw()
    }
    
    /// Get raw value of firmware_version
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_version_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of firmware_version
    #[inline(always)]
    pub fn set_firmware_version(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 131 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module1Status {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module1Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module1Status")
                .field("board_humidity", &self.board_humidity())
                .field("board_temperature", &self.board_temperature())
                .field("firmware_is_dirty", &self.firmware_is_dirty())
                .field("firmware_on_main", &self.firmware_on_main())
                .field("board_on_time_seconds", &self.board_on_time_seconds())
                .field("firmware_version", &self.firmware_version())
            .finish()
        } else {
            f.debug_tuple("Module1Status").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module1Status {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let board_humidity = u.int_in_range(0..=127)?;
        let board_temperature = u.int_in_range(0..=127)?;
        let firmware_is_dirty = u.int_in_range(0..=1)? == 1;
        let firmware_on_main = u.int_in_range(0..=1)? == 1;
        let board_on_time_seconds = u.int_in_range(0..=65535)?;
        let firmware_version = u.int_in_range(0..=4294967295)?;
        Module1Status::new(board_humidity,board_temperature,firmware_is_dirty,firmware_on_main,board_on_time_seconds,firmware_version).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module2_status
///
/// - ID: 132 (0x84)
/// - Size: 8 bytes
/// - Transmitter: module2
///
/// Git info for the firmware, on timer, along with temperature and humidity
#[derive(Clone, Copy)]
pub struct Module2Status {
    raw: [u8; 8],
}

impl Module2Status {
    pub const MESSAGE_ID: u32 = 132;
    
    pub const BOARD_HUMIDITY_MIN: u8 = 0_u8;
    pub const BOARD_HUMIDITY_MAX: u8 = 127_u8;
    pub const BOARD_TEMPERATURE_MIN: u8 = 0_u8;
    pub const BOARD_TEMPERATURE_MAX: u8 = 127_u8;
    pub const BOARD_ON_TIME_SECONDS_MIN: u16 = 0_u16;
    pub const BOARD_ON_TIME_SECONDS_MAX: u16 = 65535_u16;
    pub const FIRMWARE_VERSION_MIN: u32 = 0_u32;
    pub const FIRMWARE_VERSION_MAX: u32 = 4294967295_u32;
    
    /// Construct new module2_status from values
    pub fn new(board_humidity: u8, board_temperature: u8, firmware_is_dirty: bool, firmware_on_main: bool, board_on_time_seconds: u16, firmware_version: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_board_humidity(board_humidity)?;
        res.set_board_temperature(board_temperature)?;
        res.set_firmware_is_dirty(firmware_is_dirty)?;
        res.set_firmware_on_main(firmware_on_main)?;
        res.set_board_on_time_seconds(board_on_time_seconds)?;
        res.set_firmware_version(firmware_version)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// board_humidity
    ///
    /// the relative humidity at the acu
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "relative_humidity"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_humidity(&self) -> u8 {
        self.board_humidity_raw()
    }
    
    /// Get raw value of board_humidity
    ///
    /// - Start bit: 57
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_humidity_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[57..64].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_humidity
    #[inline(always)]
    pub fn set_board_humidity(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 132 });
        }
        self.raw.view_bits_mut::<Lsb0>()[57..64].store_le(value);
        Ok(())
    }
    
    /// board_temperature
    ///
    /// The ambient temperature at the PCB
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "celcius"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_temperature(&self) -> u8 {
        self.board_temperature_raw()
    }
    
    /// Get raw value of board_temperature
    ///
    /// - Start bit: 50
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[50..57].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_temperature
    #[inline(always)]
    pub fn set_board_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 132 });
        }
        self.raw.view_bits_mut::<Lsb0>()[50..57].store_le(value);
        Ok(())
    }
    
    /// firmware_is_dirty
    ///
    /// if the compiled firmware had uncommitted changes (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_is_dirty(&self) -> bool {
        self.firmware_is_dirty_raw()
    }
    
    /// Get raw value of firmware_is_dirty
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_is_dirty_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_is_dirty
    #[inline(always)]
    pub fn set_firmware_is_dirty(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// firmware_on_main
    ///
    /// if the compiled firmware was from main branch (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_on_main(&self) -> bool {
        self.firmware_on_main_raw()
    }
    
    /// Get raw value of firmware_on_main
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_on_main_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_on_main
    #[inline(always)]
    pub fn set_firmware_on_main(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// board_on_time_seconds
    ///
    /// time in seconds which the board has been powered on and running
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_on_time_seconds(&self) -> u16 {
        self.board_on_time_seconds_raw()
    }
    
    /// Get raw value of board_on_time_seconds
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_on_time_seconds_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of board_on_time_seconds
    #[inline(always)]
    pub fn set_board_on_time_seconds(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 132 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// firmware_version
    ///
    /// short hash of the firmware
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version_raw()
    }
    
    /// Get raw value of firmware_version
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_version_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of firmware_version
    #[inline(always)]
    pub fn set_firmware_version(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 132 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module2Status {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module2Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module2Status")
                .field("board_humidity", &self.board_humidity())
                .field("board_temperature", &self.board_temperature())
                .field("firmware_is_dirty", &self.firmware_is_dirty())
                .field("firmware_on_main", &self.firmware_on_main())
                .field("board_on_time_seconds", &self.board_on_time_seconds())
                .field("firmware_version", &self.firmware_version())
            .finish()
        } else {
            f.debug_tuple("Module2Status").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module2Status {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let board_humidity = u.int_in_range(0..=127)?;
        let board_temperature = u.int_in_range(0..=127)?;
        let firmware_is_dirty = u.int_in_range(0..=1)? == 1;
        let firmware_on_main = u.int_in_range(0..=1)? == 1;
        let board_on_time_seconds = u.int_in_range(0..=65535)?;
        let firmware_version = u.int_in_range(0..=4294967295)?;
        Module2Status::new(board_humidity,board_temperature,firmware_is_dirty,firmware_on_main,board_on_time_seconds,firmware_version).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module3_status
///
/// - ID: 133 (0x85)
/// - Size: 8 bytes
/// - Transmitter: module3
///
/// Git info for the firmware, on timer, along with temperature and humidity
#[derive(Clone, Copy)]
pub struct Module3Status {
    raw: [u8; 8],
}

impl Module3Status {
    pub const MESSAGE_ID: u32 = 133;
    
    pub const BOARD_HUMIDITY_MIN: u8 = 0_u8;
    pub const BOARD_HUMIDITY_MAX: u8 = 127_u8;
    pub const BOARD_TEMPERATURE_MIN: u8 = 0_u8;
    pub const BOARD_TEMPERATURE_MAX: u8 = 127_u8;
    pub const BOARD_ON_TIME_SECONDS_MIN: u16 = 0_u16;
    pub const BOARD_ON_TIME_SECONDS_MAX: u16 = 65535_u16;
    pub const FIRMWARE_VERSION_MIN: u32 = 0_u32;
    pub const FIRMWARE_VERSION_MAX: u32 = 4294967295_u32;
    
    /// Construct new module3_status from values
    pub fn new(board_humidity: u8, board_temperature: u8, firmware_is_dirty: bool, firmware_on_main: bool, board_on_time_seconds: u16, firmware_version: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_board_humidity(board_humidity)?;
        res.set_board_temperature(board_temperature)?;
        res.set_firmware_is_dirty(firmware_is_dirty)?;
        res.set_firmware_on_main(firmware_on_main)?;
        res.set_board_on_time_seconds(board_on_time_seconds)?;
        res.set_firmware_version(firmware_version)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// board_humidity
    ///
    /// the relative humidity at the acu
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "relative_humidity"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_humidity(&self) -> u8 {
        self.board_humidity_raw()
    }
    
    /// Get raw value of board_humidity
    ///
    /// - Start bit: 57
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_humidity_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[57..64].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_humidity
    #[inline(always)]
    pub fn set_board_humidity(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 133 });
        }
        self.raw.view_bits_mut::<Lsb0>()[57..64].store_le(value);
        Ok(())
    }
    
    /// board_temperature
    ///
    /// The ambient temperature at the PCB
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "celcius"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_temperature(&self) -> u8 {
        self.board_temperature_raw()
    }
    
    /// Get raw value of board_temperature
    ///
    /// - Start bit: 50
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[50..57].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_temperature
    #[inline(always)]
    pub fn set_board_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 133 });
        }
        self.raw.view_bits_mut::<Lsb0>()[50..57].store_le(value);
        Ok(())
    }
    
    /// firmware_is_dirty
    ///
    /// if the compiled firmware had uncommitted changes (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_is_dirty(&self) -> bool {
        self.firmware_is_dirty_raw()
    }
    
    /// Get raw value of firmware_is_dirty
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_is_dirty_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_is_dirty
    #[inline(always)]
    pub fn set_firmware_is_dirty(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// firmware_on_main
    ///
    /// if the compiled firmware was from main branch (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_on_main(&self) -> bool {
        self.firmware_on_main_raw()
    }
    
    /// Get raw value of firmware_on_main
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_on_main_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_on_main
    #[inline(always)]
    pub fn set_firmware_on_main(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// board_on_time_seconds
    ///
    /// time in seconds which the board has been powered on and running
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_on_time_seconds(&self) -> u16 {
        self.board_on_time_seconds_raw()
    }
    
    /// Get raw value of board_on_time_seconds
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_on_time_seconds_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of board_on_time_seconds
    #[inline(always)]
    pub fn set_board_on_time_seconds(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 133 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// firmware_version
    ///
    /// short hash of the firmware
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version_raw()
    }
    
    /// Get raw value of firmware_version
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_version_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of firmware_version
    #[inline(always)]
    pub fn set_firmware_version(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 133 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module3Status {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module3Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module3Status")
                .field("board_humidity", &self.board_humidity())
                .field("board_temperature", &self.board_temperature())
                .field("firmware_is_dirty", &self.firmware_is_dirty())
                .field("firmware_on_main", &self.firmware_on_main())
                .field("board_on_time_seconds", &self.board_on_time_seconds())
                .field("firmware_version", &self.firmware_version())
            .finish()
        } else {
            f.debug_tuple("Module3Status").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module3Status {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let board_humidity = u.int_in_range(0..=127)?;
        let board_temperature = u.int_in_range(0..=127)?;
        let firmware_is_dirty = u.int_in_range(0..=1)? == 1;
        let firmware_on_main = u.int_in_range(0..=1)? == 1;
        let board_on_time_seconds = u.int_in_range(0..=65535)?;
        let firmware_version = u.int_in_range(0..=4294967295)?;
        Module3Status::new(board_humidity,board_temperature,firmware_is_dirty,firmware_on_main,board_on_time_seconds,firmware_version).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module4_status
///
/// - ID: 134 (0x86)
/// - Size: 8 bytes
/// - Transmitter: module4
///
/// Git info for the firmware, on timer, along with temperature and humidity
#[derive(Clone, Copy)]
pub struct Module4Status {
    raw: [u8; 8],
}

impl Module4Status {
    pub const MESSAGE_ID: u32 = 134;
    
    pub const BOARD_HUMIDITY_MIN: u8 = 0_u8;
    pub const BOARD_HUMIDITY_MAX: u8 = 127_u8;
    pub const BOARD_TEMPERATURE_MIN: u8 = 0_u8;
    pub const BOARD_TEMPERATURE_MAX: u8 = 127_u8;
    pub const BOARD_ON_TIME_SECONDS_MIN: u16 = 0_u16;
    pub const BOARD_ON_TIME_SECONDS_MAX: u16 = 65535_u16;
    pub const FIRMWARE_VERSION_MIN: u32 = 0_u32;
    pub const FIRMWARE_VERSION_MAX: u32 = 4294967295_u32;
    
    /// Construct new module4_status from values
    pub fn new(board_humidity: u8, board_temperature: u8, firmware_is_dirty: bool, firmware_on_main: bool, board_on_time_seconds: u16, firmware_version: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_board_humidity(board_humidity)?;
        res.set_board_temperature(board_temperature)?;
        res.set_firmware_is_dirty(firmware_is_dirty)?;
        res.set_firmware_on_main(firmware_on_main)?;
        res.set_board_on_time_seconds(board_on_time_seconds)?;
        res.set_firmware_version(firmware_version)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// board_humidity
    ///
    /// the relative humidity at the acu
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "relative_humidity"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_humidity(&self) -> u8 {
        self.board_humidity_raw()
    }
    
    /// Get raw value of board_humidity
    ///
    /// - Start bit: 57
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_humidity_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[57..64].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_humidity
    #[inline(always)]
    pub fn set_board_humidity(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 134 });
        }
        self.raw.view_bits_mut::<Lsb0>()[57..64].store_le(value);
        Ok(())
    }
    
    /// board_temperature
    ///
    /// The ambient temperature at the PCB
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "celcius"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_temperature(&self) -> u8 {
        self.board_temperature_raw()
    }
    
    /// Get raw value of board_temperature
    ///
    /// - Start bit: 50
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[50..57].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_temperature
    #[inline(always)]
    pub fn set_board_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 134 });
        }
        self.raw.view_bits_mut::<Lsb0>()[50..57].store_le(value);
        Ok(())
    }
    
    /// firmware_is_dirty
    ///
    /// if the compiled firmware had uncommitted changes (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_is_dirty(&self) -> bool {
        self.firmware_is_dirty_raw()
    }
    
    /// Get raw value of firmware_is_dirty
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_is_dirty_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_is_dirty
    #[inline(always)]
    pub fn set_firmware_is_dirty(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// firmware_on_main
    ///
    /// if the compiled firmware was from main branch (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_on_main(&self) -> bool {
        self.firmware_on_main_raw()
    }
    
    /// Get raw value of firmware_on_main
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_on_main_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_on_main
    #[inline(always)]
    pub fn set_firmware_on_main(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// board_on_time_seconds
    ///
    /// time in seconds which the board has been powered on and running
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_on_time_seconds(&self) -> u16 {
        self.board_on_time_seconds_raw()
    }
    
    /// Get raw value of board_on_time_seconds
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_on_time_seconds_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of board_on_time_seconds
    #[inline(always)]
    pub fn set_board_on_time_seconds(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 134 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// firmware_version
    ///
    /// short hash of the firmware
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version_raw()
    }
    
    /// Get raw value of firmware_version
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_version_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of firmware_version
    #[inline(always)]
    pub fn set_firmware_version(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 134 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module4Status {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module4Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module4Status")
                .field("board_humidity", &self.board_humidity())
                .field("board_temperature", &self.board_temperature())
                .field("firmware_is_dirty", &self.firmware_is_dirty())
                .field("firmware_on_main", &self.firmware_on_main())
                .field("board_on_time_seconds", &self.board_on_time_seconds())
                .field("firmware_version", &self.firmware_version())
            .finish()
        } else {
            f.debug_tuple("Module4Status").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module4Status {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let board_humidity = u.int_in_range(0..=127)?;
        let board_temperature = u.int_in_range(0..=127)?;
        let firmware_is_dirty = u.int_in_range(0..=1)? == 1;
        let firmware_on_main = u.int_in_range(0..=1)? == 1;
        let board_on_time_seconds = u.int_in_range(0..=65535)?;
        let firmware_version = u.int_in_range(0..=4294967295)?;
        Module4Status::new(board_humidity,board_temperature,firmware_is_dirty,firmware_on_main,board_on_time_seconds,firmware_version).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// module5_status
///
/// - ID: 135 (0x87)
/// - Size: 8 bytes
/// - Transmitter: module5
///
/// Git info for the firmware, on timer, along with temperature and humidity
#[derive(Clone, Copy)]
pub struct Module5Status {
    raw: [u8; 8],
}

impl Module5Status {
    pub const MESSAGE_ID: u32 = 135;
    
    pub const BOARD_HUMIDITY_MIN: u8 = 0_u8;
    pub const BOARD_HUMIDITY_MAX: u8 = 127_u8;
    pub const BOARD_TEMPERATURE_MIN: u8 = 0_u8;
    pub const BOARD_TEMPERATURE_MAX: u8 = 127_u8;
    pub const BOARD_ON_TIME_SECONDS_MIN: u16 = 0_u16;
    pub const BOARD_ON_TIME_SECONDS_MAX: u16 = 65535_u16;
    pub const FIRMWARE_VERSION_MIN: u32 = 0_u32;
    pub const FIRMWARE_VERSION_MAX: u32 = 4294967295_u32;
    
    /// Construct new module5_status from values
    pub fn new(board_humidity: u8, board_temperature: u8, firmware_is_dirty: bool, firmware_on_main: bool, board_on_time_seconds: u16, firmware_version: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_board_humidity(board_humidity)?;
        res.set_board_temperature(board_temperature)?;
        res.set_firmware_is_dirty(firmware_is_dirty)?;
        res.set_firmware_on_main(firmware_on_main)?;
        res.set_board_on_time_seconds(board_on_time_seconds)?;
        res.set_firmware_version(firmware_version)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// board_humidity
    ///
    /// the relative humidity at the acu
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "relative_humidity"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_humidity(&self) -> u8 {
        self.board_humidity_raw()
    }
    
    /// Get raw value of board_humidity
    ///
    /// - Start bit: 57
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_humidity_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[57..64].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_humidity
    #[inline(always)]
    pub fn set_board_humidity(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 135 });
        }
        self.raw.view_bits_mut::<Lsb0>()[57..64].store_le(value);
        Ok(())
    }
    
    /// board_temperature
    ///
    /// The ambient temperature at the PCB
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "celcius"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_temperature(&self) -> u8 {
        self.board_temperature_raw()
    }
    
    /// Get raw value of board_temperature
    ///
    /// - Start bit: 50
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[50..57].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_temperature
    #[inline(always)]
    pub fn set_board_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 135 });
        }
        self.raw.view_bits_mut::<Lsb0>()[50..57].store_le(value);
        Ok(())
    }
    
    /// firmware_is_dirty
    ///
    /// if the compiled firmware had uncommitted changes (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_is_dirty(&self) -> bool {
        self.firmware_is_dirty_raw()
    }
    
    /// Get raw value of firmware_is_dirty
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_is_dirty_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_is_dirty
    #[inline(always)]
    pub fn set_firmware_is_dirty(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// firmware_on_main
    ///
    /// if the compiled firmware was from main branch (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_on_main(&self) -> bool {
        self.firmware_on_main_raw()
    }
    
    /// Get raw value of firmware_on_main
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_on_main_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_on_main
    #[inline(always)]
    pub fn set_firmware_on_main(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// board_on_time_seconds
    ///
    /// time in seconds which the board has been powered on and running
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_on_time_seconds(&self) -> u16 {
        self.board_on_time_seconds_raw()
    }
    
    /// Get raw value of board_on_time_seconds
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_on_time_seconds_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of board_on_time_seconds
    #[inline(always)]
    pub fn set_board_on_time_seconds(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 135 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// firmware_version
    ///
    /// short hash of the firmware
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version_raw()
    }
    
    /// Get raw value of firmware_version
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_version_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of firmware_version
    #[inline(always)]
    pub fn set_firmware_version(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 135 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Module5Status {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Module5Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Module5Status")
                .field("board_humidity", &self.board_humidity())
                .field("board_temperature", &self.board_temperature())
                .field("firmware_is_dirty", &self.firmware_is_dirty())
                .field("firmware_on_main", &self.firmware_on_main())
                .field("board_on_time_seconds", &self.board_on_time_seconds())
                .field("firmware_version", &self.firmware_version())
            .finish()
        } else {
            f.debug_tuple("Module5Status").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Module5Status {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let board_humidity = u.int_in_range(0..=127)?;
        let board_temperature = u.int_in_range(0..=127)?;
        let firmware_is_dirty = u.int_in_range(0..=1)? == 1;
        let firmware_on_main = u.int_in_range(0..=1)? == 1;
        let board_on_time_seconds = u.int_in_range(0..=65535)?;
        let firmware_version = u.int_in_range(0..=4294967295)?;
        Module5Status::new(board_humidity,board_temperature,firmware_is_dirty,firmware_on_main,board_on_time_seconds,firmware_version).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// VCU_STATUS
///
/// - ID: 195 (0xc3)
/// - Size: 7 bytes
/// - Transmitter: vcu
///
/// Vehicle Control Unit Status message
#[derive(Clone, Copy)]
pub struct VcuStatus {
    raw: [u8; 7],
}

impl VcuStatus {
    pub const MESSAGE_ID: u32 = 195;
    
    pub const VCU_DISTANCE_TRAVELLED_MIN: u16 = 0_u16;
    pub const VCU_DISTANCE_TRAVELLED_MAX: u16 = 65535_u16;
    pub const VCU_TORQUE_MODE_MIN: u8 = 0_u8;
    pub const VCU_TORQUE_MODE_MAX: u8 = 255_u8;
    pub const VCU_MAX_TORQUE_MIN: u8 = 0_u8;
    pub const VCU_MAX_TORQUE_MAX: u8 = 255_u8;
    pub const VCU_STATEMACHINE_STATE_MIN: u8 = 0_u8;
    pub const VCU_STATEMACHINE_STATE_MAX: u8 = 7_u8;
    
    /// Construct new VCU_STATUS from values
    pub fn new(vcu_distance_travelled: u16, vcu_torque_mode: u8, vcu_max_torque: u8, vcu_launch_control_active: bool, vcu_software_ok: bool, vcu_activate_buzzer: bool, vcu_energy_meter_present: bool, vcu_inverter_powered: bool, vcu_statemachine_state: u8, vcu_accel_brake_implausible: bool, vcu_bspd_brake_high: bool, vcu_bspd_current_high: bool, vcu_brake_active: bool, vcu_brake_implausible: bool, vcu_accel_implausible: bool, vcu_shutdown_e_ok_high: bool, vcu_software_ok_high: bool, vcu_shutdown_d_ok_high: bool, vcu_bspd_ok_high: bool, vcu_shutdown_c_ok_high: bool, vcu_bms_ok_high: bool, vcu_shutdown_b_ok_high: bool, vcu_imd_ok_high: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 7] };
        res.set_vcu_distance_travelled(vcu_distance_travelled)?;
        res.set_vcu_torque_mode(vcu_torque_mode)?;
        res.set_vcu_max_torque(vcu_max_torque)?;
        res.set_vcu_launch_control_active(vcu_launch_control_active)?;
        res.set_vcu_software_ok(vcu_software_ok)?;
        res.set_vcu_activate_buzzer(vcu_activate_buzzer)?;
        res.set_vcu_energy_meter_present(vcu_energy_meter_present)?;
        res.set_vcu_inverter_powered(vcu_inverter_powered)?;
        res.set_vcu_statemachine_state(vcu_statemachine_state)?;
        res.set_vcu_accel_brake_implausible(vcu_accel_brake_implausible)?;
        res.set_vcu_bspd_brake_high(vcu_bspd_brake_high)?;
        res.set_vcu_bspd_current_high(vcu_bspd_current_high)?;
        res.set_vcu_brake_active(vcu_brake_active)?;
        res.set_vcu_brake_implausible(vcu_brake_implausible)?;
        res.set_vcu_accel_implausible(vcu_accel_implausible)?;
        res.set_vcu_shutdown_e_ok_high(vcu_shutdown_e_ok_high)?;
        res.set_vcu_software_ok_high(vcu_software_ok_high)?;
        res.set_vcu_shutdown_d_ok_high(vcu_shutdown_d_ok_high)?;
        res.set_vcu_bspd_ok_high(vcu_bspd_ok_high)?;
        res.set_vcu_shutdown_c_ok_high(vcu_shutdown_c_ok_high)?;
        res.set_vcu_bms_ok_high(vcu_bms_ok_high)?;
        res.set_vcu_shutdown_b_ok_high(vcu_shutdown_b_ok_high)?;
        res.set_vcu_imd_ok_high(vcu_imd_ok_high)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 7] {
        &self.raw
    }
    
    /// VCU_DISTANCE_TRAVELLED
    ///
    /// distance travelled in current driving session
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "meters"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_distance_travelled(&self) -> u16 {
        self.vcu_distance_travelled_raw()
    }
    
    /// Get raw value of VCU_DISTANCE_TRAVELLED
    ///
    /// - Start bit: 40
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_distance_travelled_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[40..56].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_DISTANCE_TRAVELLED
    #[inline(always)]
    pub fn set_vcu_distance_travelled(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 195 });
        }
        self.raw.view_bits_mut::<Lsb0>()[40..56].store_le(value);
        Ok(())
    }
    
    /// VCU_TORQUE_MODE
    ///
    /// VCU torque mode setting
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_torque_mode(&self) -> u8 {
        self.vcu_torque_mode_raw()
    }
    
    /// Get raw value of VCU_TORQUE_MODE
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_torque_mode_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        signal
    }
    
    /// Set value of VCU_TORQUE_MODE
    #[inline(always)]
    pub fn set_vcu_torque_mode(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 195 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// VCU_MAX_TORQUE
    ///
    /// max torque setting in Newton-meters
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_max_torque(&self) -> u8 {
        self.vcu_max_torque_raw()
    }
    
    /// Get raw value of VCU_MAX_TORQUE
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_max_torque_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        signal
    }
    
    /// Set value of VCU_MAX_TORQUE
    #[inline(always)]
    pub fn set_vcu_max_torque(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 195 });
        }
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// VCU_LAUNCH_CONTROL_ACTIVE
    ///
    /// 1 if launch control is active, 0 if not
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_launch_control_active(&self) -> bool {
        self.vcu_launch_control_active_raw()
    }
    
    /// Get raw value of VCU_LAUNCH_CONTROL_ACTIVE
    ///
    /// - Start bit: 23
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_launch_control_active_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[23..24].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_LAUNCH_CONTROL_ACTIVE
    #[inline(always)]
    pub fn set_vcu_launch_control_active(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[23..24].store_le(value);
        Ok(())
    }
    
    /// VCU_SOFTWARE_OK
    ///
    /// currently unused
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_software_ok(&self) -> bool {
        self.vcu_software_ok_raw()
    }
    
    /// Get raw value of VCU_SOFTWARE_OK
    ///
    /// - Start bit: 22
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_software_ok_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[22..23].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_SOFTWARE_OK
    #[inline(always)]
    pub fn set_vcu_software_ok(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[22..23].store_le(value);
        Ok(())
    }
    
    /// VCU_ACTIVATE_BUZZER
    ///
    /// 1 if the buzzer is activated, 0 if not
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_activate_buzzer(&self) -> bool {
        self.vcu_activate_buzzer_raw()
    }
    
    /// Get raw value of VCU_ACTIVATE_BUZZER
    ///
    /// - Start bit: 21
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_activate_buzzer_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[21..22].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_ACTIVATE_BUZZER
    #[inline(always)]
    pub fn set_vcu_activate_buzzer(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[21..22].store_le(value);
        Ok(())
    }
    
    /// VCU_ENERGY_METER_PRESENT
    ///
    /// 1 if the energy meter is present, 0 if not
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_energy_meter_present(&self) -> bool {
        self.vcu_energy_meter_present_raw()
    }
    
    /// Get raw value of VCU_ENERGY_METER_PRESENT
    ///
    /// - Start bit: 20
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_energy_meter_present_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[20..21].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_ENERGY_METER_PRESENT
    #[inline(always)]
    pub fn set_vcu_energy_meter_present(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[20..21].store_le(value);
        Ok(())
    }
    
    /// VCU_INVERTER_POWERED
    ///
    /// 1 if the inverter is powered, 0 if not
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inverter_powered(&self) -> bool {
        self.vcu_inverter_powered_raw()
    }
    
    /// Get raw value of VCU_INVERTER_POWERED
    ///
    /// - Start bit: 19
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_inverter_powered_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[19..20].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_INVERTER_POWERED
    #[inline(always)]
    pub fn set_vcu_inverter_powered(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[19..20].store_le(value);
        Ok(())
    }
    
    /// VCU_STATEMACHINE_STATE
    ///
    /// 0 = STARTUP, 1 = TS NOT ACTIVE, 2 = TS ACTIVE, 3 = ENABLING INVERTER, 4 = WAIT RTD SOUND, 5=RTD
    ///
    /// - Min: 0
    /// - Max: 7
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_statemachine_state(&self) -> u8 {
        self.vcu_statemachine_state_raw()
    }
    
    /// Get raw value of VCU_STATEMACHINE_STATE
    ///
    /// - Start bit: 16
    /// - Signal size: 3 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_statemachine_state_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[16..19].load_le::<u8>();
        
        signal
    }
    
    /// Set value of VCU_STATEMACHINE_STATE
    #[inline(always)]
    pub fn set_vcu_statemachine_state(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 7_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 195 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..19].store_le(value);
        Ok(())
    }
    
    /// VCU_ACCEL_BRAKE_IMPLAUSIBLE
    ///
    /// 1 if the accel and brake plausibility is tripped
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_accel_brake_implausible(&self) -> bool {
        self.vcu_accel_brake_implausible_raw()
    }
    
    /// Get raw value of VCU_ACCEL_BRAKE_IMPLAUSIBLE
    ///
    /// - Start bit: 15
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_accel_brake_implausible_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[15..16].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_ACCEL_BRAKE_IMPLAUSIBLE
    #[inline(always)]
    pub fn set_vcu_accel_brake_implausible(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[15..16].store_le(value);
        Ok(())
    }
    
    /// VCU_BSPD_BRAKE_HIGH
    ///
    /// 1 if the brake is above the bspd trip threshold
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_bspd_brake_high(&self) -> bool {
        self.vcu_bspd_brake_high_raw()
    }
    
    /// Get raw value of VCU_BSPD_BRAKE_HIGH
    ///
    /// - Start bit: 14
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_bspd_brake_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[14..15].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_BSPD_BRAKE_HIGH
    #[inline(always)]
    pub fn set_vcu_bspd_brake_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[14..15].store_le(value);
        Ok(())
    }
    
    /// VCU_BSPD_CURRENT_HIGH
    ///
    /// 1 if the acc current is above the bspd trip threshold
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_bspd_current_high(&self) -> bool {
        self.vcu_bspd_current_high_raw()
    }
    
    /// Get raw value of VCU_BSPD_CURRENT_HIGH
    ///
    /// - Start bit: 13
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_bspd_current_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[13..14].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_BSPD_CURRENT_HIGH
    #[inline(always)]
    pub fn set_vcu_bspd_current_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[13..14].store_le(value);
        Ok(())
    }
    
    /// VCU_BRAKE_ACTIVE
    ///
    /// 1 if the brake is active (above 'vcu_brake_active_threshold')
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_brake_active(&self) -> bool {
        self.vcu_brake_active_raw()
    }
    
    /// Get raw value of VCU_BRAKE_ACTIVE
    ///
    /// - Start bit: 12
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_brake_active_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[12..13].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_BRAKE_ACTIVE
    #[inline(always)]
    pub fn set_vcu_brake_active(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[12..13].store_le(value);
        Ok(())
    }
    
    /// VCU_BRAKE_IMPLAUSIBLE
    ///
    /// 1 if the brake is implausible, 0 if not
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_brake_implausible(&self) -> bool {
        self.vcu_brake_implausible_raw()
    }
    
    /// Get raw value of VCU_BRAKE_IMPLAUSIBLE
    ///
    /// - Start bit: 11
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_brake_implausible_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[11..12].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_BRAKE_IMPLAUSIBLE
    #[inline(always)]
    pub fn set_vcu_brake_implausible(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[11..12].store_le(value);
        Ok(())
    }
    
    /// VCU_ACCEL_IMPLAUSIBLE
    ///
    /// 1 if the accel pedal is implausible, 0 if not
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_accel_implausible(&self) -> bool {
        self.vcu_accel_implausible_raw()
    }
    
    /// Get raw value of VCU_ACCEL_IMPLAUSIBLE
    ///
    /// - Start bit: 10
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_accel_implausible_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[10..11].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_ACCEL_IMPLAUSIBLE
    #[inline(always)]
    pub fn set_vcu_accel_implausible(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[10..11].store_le(value);
        Ok(())
    }
    
    /// VCU_SHUTDOWN_E_OK_HIGH
    ///
    /// (UNUSED) Shutdown loop voltage at point E, high = good
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_shutdown_e_ok_high(&self) -> bool {
        self.vcu_shutdown_e_ok_high_raw()
    }
    
    /// Get raw value of VCU_SHUTDOWN_E_OK_HIGH
    ///
    /// - Start bit: 7
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_shutdown_e_ok_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[7..8].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_SHUTDOWN_E_OK_HIGH
    #[inline(always)]
    pub fn set_vcu_shutdown_e_ok_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[7..8].store_le(value);
        Ok(())
    }
    
    /// VCU_SOFTWARE_OK_HIGH
    ///
    /// (UNUSED) VCU heartbeat status, high = good
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_software_ok_high(&self) -> bool {
        self.vcu_software_ok_high_raw()
    }
    
    /// Get raw value of VCU_SOFTWARE_OK_HIGH
    ///
    /// - Start bit: 6
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_software_ok_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[6..7].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_SOFTWARE_OK_HIGH
    #[inline(always)]
    pub fn set_vcu_software_ok_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[6..7].store_le(value);
        Ok(())
    }
    
    /// VCU_SHUTDOWN_D_OK_HIGH
    ///
    /// (UNUSED) Shutdown loop voltage at point D, high = good
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_shutdown_d_ok_high(&self) -> bool {
        self.vcu_shutdown_d_ok_high_raw()
    }
    
    /// Get raw value of VCU_SHUTDOWN_D_OK_HIGH
    ///
    /// - Start bit: 5
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_shutdown_d_ok_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[5..6].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_SHUTDOWN_D_OK_HIGH
    #[inline(always)]
    pub fn set_vcu_shutdown_d_ok_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[5..6].store_le(value);
        Ok(())
    }
    
    /// VCU_BSPD_OK_HIGH
    ///
    /// BSPD status, high = good (latched)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_bspd_ok_high(&self) -> bool {
        self.vcu_bspd_ok_high_raw()
    }
    
    /// Get raw value of VCU_BSPD_OK_HIGH
    ///
    /// - Start bit: 4
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_bspd_ok_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[4..5].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_BSPD_OK_HIGH
    #[inline(always)]
    pub fn set_vcu_bspd_ok_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[4..5].store_le(value);
        Ok(())
    }
    
    /// VCU_SHUTDOWN_C_OK_HIGH
    ///
    /// (UNUSED) Shutdown loop voltage at point C, high = good
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_shutdown_c_ok_high(&self) -> bool {
        self.vcu_shutdown_c_ok_high_raw()
    }
    
    /// Get raw value of VCU_SHUTDOWN_C_OK_HIGH
    ///
    /// - Start bit: 3
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_shutdown_c_ok_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[3..4].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_SHUTDOWN_C_OK_HIGH
    #[inline(always)]
    pub fn set_vcu_shutdown_c_ok_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[3..4].store_le(value);
        Ok(())
    }
    
    /// VCU_BMS_OK_HIGH
    ///
    /// BMS status, high = good
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_bms_ok_high(&self) -> bool {
        self.vcu_bms_ok_high_raw()
    }
    
    /// Get raw value of VCU_BMS_OK_HIGH
    ///
    /// - Start bit: 2
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_bms_ok_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[2..3].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_BMS_OK_HIGH
    #[inline(always)]
    pub fn set_vcu_bms_ok_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[2..3].store_le(value);
        Ok(())
    }
    
    /// VCU_SHUTDOWN_B_OK_HIGH
    ///
    /// (UNUSED) Shutdown loop voltage at point D, high = good
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_shutdown_b_ok_high(&self) -> bool {
        self.vcu_shutdown_b_ok_high_raw()
    }
    
    /// Get raw value of VCU_SHUTDOWN_B_OK_HIGH
    ///
    /// - Start bit: 1
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_shutdown_b_ok_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[1..2].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_SHUTDOWN_B_OK_HIGH
    #[inline(always)]
    pub fn set_vcu_shutdown_b_ok_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[1..2].store_le(value);
        Ok(())
    }
    
    /// VCU_IMD_OK_HIGH
    ///
    /// IMD status, high = good
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_imd_ok_high(&self) -> bool {
        self.vcu_imd_ok_high_raw()
    }
    
    /// Get raw value of VCU_IMD_OK_HIGH
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_imd_ok_high_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_IMD_OK_HIGH
    #[inline(always)]
    pub fn set_vcu_imd_ok_high(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuStatus {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 7 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&payload[..7]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuStatus")
                .field("vcu_distance_travelled", &self.vcu_distance_travelled())
                .field("vcu_torque_mode", &self.vcu_torque_mode())
                .field("vcu_max_torque", &self.vcu_max_torque())
                .field("vcu_launch_control_active", &self.vcu_launch_control_active())
                .field("vcu_software_ok", &self.vcu_software_ok())
                .field("vcu_activate_buzzer", &self.vcu_activate_buzzer())
                .field("vcu_energy_meter_present", &self.vcu_energy_meter_present())
                .field("vcu_inverter_powered", &self.vcu_inverter_powered())
                .field("vcu_statemachine_state", &self.vcu_statemachine_state())
                .field("vcu_accel_brake_implausible", &self.vcu_accel_brake_implausible())
                .field("vcu_bspd_brake_high", &self.vcu_bspd_brake_high())
                .field("vcu_bspd_current_high", &self.vcu_bspd_current_high())
                .field("vcu_brake_active", &self.vcu_brake_active())
                .field("vcu_brake_implausible", &self.vcu_brake_implausible())
                .field("vcu_accel_implausible", &self.vcu_accel_implausible())
                .field("vcu_shutdown_e_ok_high", &self.vcu_shutdown_e_ok_high())
                .field("vcu_software_ok_high", &self.vcu_software_ok_high())
                .field("vcu_shutdown_d_ok_high", &self.vcu_shutdown_d_ok_high())
                .field("vcu_bspd_ok_high", &self.vcu_bspd_ok_high())
                .field("vcu_shutdown_c_ok_high", &self.vcu_shutdown_c_ok_high())
                .field("vcu_bms_ok_high", &self.vcu_bms_ok_high())
                .field("vcu_shutdown_b_ok_high", &self.vcu_shutdown_b_ok_high())
                .field("vcu_imd_ok_high", &self.vcu_imd_ok_high())
            .finish()
        } else {
            f.debug_tuple("VcuStatus").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuStatus {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_distance_travelled = u.int_in_range(0..=65535)?;
        let vcu_torque_mode = u.int_in_range(0..=255)?;
        let vcu_max_torque = u.int_in_range(0..=255)?;
        let vcu_launch_control_active = u.int_in_range(0..=1)? == 1;
        let vcu_software_ok = u.int_in_range(0..=1)? == 1;
        let vcu_activate_buzzer = u.int_in_range(0..=1)? == 1;
        let vcu_energy_meter_present = u.int_in_range(0..=1)? == 1;
        let vcu_inverter_powered = u.int_in_range(0..=1)? == 1;
        let vcu_statemachine_state = u.int_in_range(0..=7)?;
        let vcu_accel_brake_implausible = u.int_in_range(0..=1)? == 1;
        let vcu_bspd_brake_high = u.int_in_range(0..=1)? == 1;
        let vcu_bspd_current_high = u.int_in_range(0..=1)? == 1;
        let vcu_brake_active = u.int_in_range(0..=1)? == 1;
        let vcu_brake_implausible = u.int_in_range(0..=1)? == 1;
        let vcu_accel_implausible = u.int_in_range(0..=1)? == 1;
        let vcu_shutdown_e_ok_high = u.int_in_range(0..=1)? == 1;
        let vcu_software_ok_high = u.int_in_range(0..=1)? == 1;
        let vcu_shutdown_d_ok_high = u.int_in_range(0..=1)? == 1;
        let vcu_bspd_ok_high = u.int_in_range(0..=1)? == 1;
        let vcu_shutdown_c_ok_high = u.int_in_range(0..=1)? == 1;
        let vcu_bms_ok_high = u.int_in_range(0..=1)? == 1;
        let vcu_shutdown_b_ok_high = u.int_in_range(0..=1)? == 1;
        let vcu_imd_ok_high = u.int_in_range(0..=1)? == 1;
        VcuStatus::new(vcu_distance_travelled,vcu_torque_mode,vcu_max_torque,vcu_launch_control_active,vcu_software_ok,vcu_activate_buzzer,vcu_energy_meter_present,vcu_inverter_powered,vcu_statemachine_state,vcu_accel_brake_implausible,vcu_bspd_brake_high,vcu_bspd_current_high,vcu_brake_active,vcu_brake_implausible,vcu_accel_implausible,vcu_shutdown_e_ok_high,vcu_software_ok_high,vcu_shutdown_d_ok_high,vcu_bspd_ok_high,vcu_shutdown_c_ok_high,vcu_bms_ok_high,vcu_shutdown_b_ok_high,vcu_imd_ok_high).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// VCU_PEDAL_READINGS
///
/// - ID: 196 (0xc4)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// VCU analog pedal readings
#[derive(Clone, Copy)]
pub struct VcuPedalReadings {
    raw: [u8; 8],
}

impl VcuPedalReadings {
    pub const MESSAGE_ID: u32 = 196;
    
    pub const STEERING_MIN: u16 = 0_u16;
    pub const STEERING_MAX: u16 = 65535_u16;
    pub const BSE1_MIN: u16 = 0_u16;
    pub const BSE1_MAX: u16 = 65535_u16;
    pub const APPS2_MIN: u16 = 0_u16;
    pub const APPS2_MAX: u16 = 65535_u16;
    pub const APPS1_MIN: u16 = 0_u16;
    pub const APPS1_MAX: u16 = 65535_u16;
    
    /// Construct new VCU_PEDAL_READINGS from values
    pub fn new(steering: u16, bse1: u16, apps2: u16, apps1: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_steering(steering)?;
        res.set_bse1(bse1)?;
        res.set_apps2(apps2)?;
        res.set_apps1(apps1)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// STEERING
    ///
    /// steering angle sensor ADC reading
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "raw"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn steering(&self) -> u16 {
        self.steering_raw()
    }
    
    /// Get raw value of STEERING
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn steering_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        signal
    }
    
    /// Set value of STEERING
    #[inline(always)]
    pub fn set_steering(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 196 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// BSE1
    ///
    /// brake pedal travel sensor ADC reading
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "raw"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn bse1(&self) -> u16 {
        self.bse1_raw()
    }
    
    /// Get raw value of BSE1
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn bse1_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of BSE1
    #[inline(always)]
    pub fn set_bse1(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 196 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// APPS2
    ///
    /// accelerator pedal sensor 2 ADC reading
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "raw"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn apps2(&self) -> u16 {
        self.apps2_raw()
    }
    
    /// Get raw value of APPS2
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn apps2_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of APPS2
    #[inline(always)]
    pub fn set_apps2(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 196 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// APPS1
    ///
    /// accelerator pedal sensor 1 ADC reading
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "raw"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn apps1(&self) -> u16 {
        self.apps1_raw()
    }
    
    /// Get raw value of APPS1
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn apps1_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of APPS1
    #[inline(always)]
    pub fn set_apps1(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 196 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuPedalReadings {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuPedalReadings {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuPedalReadings")
                .field("steering", &self.steering())
                .field("bse1", &self.bse1())
                .field("apps2", &self.apps2())
                .field("apps1", &self.apps1())
            .finish()
        } else {
            f.debug_tuple("VcuPedalReadings").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuPedalReadings {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let steering = u.int_in_range(0..=65535)?;
        let bse1 = u.int_in_range(0..=65535)?;
        let apps2 = u.int_in_range(0..=65535)?;
        let apps1 = u.int_in_range(0..=65535)?;
        VcuPedalReadings::new(steering,bse1,apps2,apps1).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// VCU_WHEELSPEED_READINGS
///
/// - ID: 198 (0xc6)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// VCU wheel speed sensor readings
#[derive(Clone, Copy)]
pub struct VcuWheelspeedReadings {
    raw: [u8; 8],
}

impl VcuWheelspeedReadings {
    pub const MESSAGE_ID: u32 = 198;
    
    pub const VCU_RPM_FRONT_RIGHT_MIN: i16 = -32768_i16;
    pub const VCU_RPM_FRONT_RIGHT_MAX: i16 = 32767_i16;
    pub const VCU_RPM_REAR_LEFT_MIN: i16 = -32768_i16;
    pub const VCU_RPM_REAR_LEFT_MAX: i16 = 32767_i16;
    
    /// Construct new VCU_WHEELSPEED_READINGS from values
    pub fn new(vcu_rpm_front_right: i16, vcu_rpm_rear_left: i16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_rpm_front_right(vcu_rpm_front_right)?;
        res.set_vcu_rpm_rear_left(vcu_rpm_rear_left)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// vcu_rpm_front_right
    ///
    /// front right wheel speed sensor
    ///
    /// - Min: -32768
    /// - Max: 32767
    /// - Unit: "rpm"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_rpm_front_right(&self) -> i16 {
        self.vcu_rpm_front_right_raw()
    }
    
    /// Get raw value of vcu_rpm_front_right
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_rpm_front_right_raw(&self) -> i16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of vcu_rpm_front_right
    #[inline(always)]
    pub fn set_vcu_rpm_front_right(&mut self, value: i16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -32768_i16 || 32767_i16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 198 });
        }
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// vcu_rpm_rear_left
    ///
    /// front left wheel speed sensor
    ///
    /// - Min: -32768
    /// - Max: 32767
    /// - Unit: "rpm"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_rpm_rear_left(&self) -> i16 {
        self.vcu_rpm_rear_left_raw()
    }
    
    /// Get raw value of vcu_rpm_rear_left
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_rpm_rear_left_raw(&self) -> i16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of vcu_rpm_rear_left
    #[inline(always)]
    pub fn set_vcu_rpm_rear_left(&mut self, value: i16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -32768_i16 || 32767_i16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 198 });
        }
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuWheelspeedReadings {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuWheelspeedReadings {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuWheelspeedReadings")
                .field("vcu_rpm_front_right", &self.vcu_rpm_front_right())
                .field("vcu_rpm_rear_left", &self.vcu_rpm_rear_left())
            .finish()
        } else {
            f.debug_tuple("VcuWheelspeedReadings").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuWheelspeedReadings {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_rpm_front_right = u.int_in_range(-32768..=32767)?;
        let vcu_rpm_rear_left = u.int_in_range(-32768..=32767)?;
        VcuWheelspeedReadings::new(vcu_rpm_front_right,vcu_rpm_rear_left).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_pedal_threshold_settings
///
/// - ID: 199 (0xc7)
/// - Size: 7 bytes
/// - Transmitter: vcu
#[derive(Clone, Copy)]
pub struct VcuPedalThresholdSettings {
    raw: [u8; 7],
}

impl VcuPedalThresholdSettings {
    pub const MESSAGE_ID: u32 = 199;
    
    pub const VCU_APPS2_OV_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_APPS2_OV_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_APPS2_UV_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_APPS2_UV_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_APPS1_START_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_APPS1_START_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_APPS2_END_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_APPS2_END_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_APPS1_OV_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_APPS1_OV_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_APPS1_UV_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_APPS1_UV_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_APPS2_START_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_APPS2_START_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_APPS1_END_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_APPS1_END_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_BRAKE_ACTIVE_THRESHOLD_MIN: u16 = 0_u16;
    pub const VCU_BRAKE_ACTIVE_THRESHOLD_MAX: u16 = 65535_u16;
    pub const VCU_PEDALS_SETTINGS_MUX_MIN: u8 = 0_u8;
    pub const VCU_PEDALS_SETTINGS_MUX_MAX: u8 = 255_u8;
    
    /// Construct new vcu_pedal_threshold_settings from values
    pub fn new(vcu_pedals_settings_mux: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 7] };
        res.set_vcu_pedals_settings_mux(vcu_pedals_settings_mux)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 7] {
        &self.raw
    }
    
    /// Get raw value of vcu_pedals_settings_MUX
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_pedals_settings_mux_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    pub fn vcu_pedals_settings_mux(&mut self) -> Result<VcuPedalThresholdSettingsVcuPedalsSettingsMux, CanError> {
        match self.vcu_pedals_settings_mux_raw() {
            0 => Ok(VcuPedalThresholdSettingsVcuPedalsSettingsMux::M0(VcuPedalThresholdSettingsVcuPedalsSettingsMuxM0{ raw: self.raw })),
            1 => Ok(VcuPedalThresholdSettingsVcuPedalsSettingsMux::M1(VcuPedalThresholdSettingsVcuPedalsSettingsMuxM1{ raw: self.raw })),
            2 => Ok(VcuPedalThresholdSettingsVcuPedalsSettingsMux::M2(VcuPedalThresholdSettingsVcuPedalsSettingsMuxM2{ raw: self.raw })),
            multiplexor => Err(CanError::InvalidMultiplexor { message_id: 199, multiplexor: multiplexor.into() }),
        }
    }
    /// Set value of vcu_pedals_settings_MUX
    #[inline(always)]
    fn set_vcu_pedals_settings_mux(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 199 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
    /// Set value of vcu_pedals_settings_MUX
    #[inline(always)]
    pub fn set_m0(&mut self, value: VcuPedalThresholdSettingsVcuPedalsSettingsMuxM0) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_vcu_pedals_settings_mux(0)?;
        Ok(())
    }
    
    /// Set value of vcu_pedals_settings_MUX
    #[inline(always)]
    pub fn set_m1(&mut self, value: VcuPedalThresholdSettingsVcuPedalsSettingsMuxM1) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_vcu_pedals_settings_mux(1)?;
        Ok(())
    }
    
    /// Set value of vcu_pedals_settings_MUX
    #[inline(always)]
    pub fn set_m2(&mut self, value: VcuPedalThresholdSettingsVcuPedalsSettingsMuxM2) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_vcu_pedals_settings_mux(2)?;
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuPedalThresholdSettings {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 7 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&payload[..7]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuPedalThresholdSettings {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuPedalThresholdSettings")
            .finish()
        } else {
            f.debug_tuple("VcuPedalThresholdSettings").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuPedalThresholdSettings {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_pedals_settings_mux = u.int_in_range(0..=255)?;
        VcuPedalThresholdSettings::new(vcu_pedals_settings_mux).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for multiplexed signal vcu_pedal_threshold_settings
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum VcuPedalThresholdSettingsVcuPedalsSettingsMux {
    M0(VcuPedalThresholdSettingsVcuPedalsSettingsMuxM0),
    M1(VcuPedalThresholdSettingsVcuPedalsSettingsMuxM1),
    M2(VcuPedalThresholdSettingsVcuPedalsSettingsMuxM2),
}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct VcuPedalThresholdSettingsVcuPedalsSettingsMuxM0 { raw: [u8; 7] }

impl VcuPedalThresholdSettingsVcuPedalsSettingsMuxM0 {
pub fn new() -> Self { Self { raw: [0u8; 7] } }
/// vcu_apps1_start_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_apps1_start_threshold(&self) -> u16 {
    self.vcu_apps1_start_threshold_raw()
}

/// Get raw value of vcu_apps1_start_threshold
///
/// - Start bit: 40
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_apps1_start_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[40..56].load_le::<u16>();
    
    signal
}

/// Set value of vcu_apps1_start_threshold
#[inline(always)]
pub fn set_vcu_apps1_start_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[40..56].store_le(value);
    Ok(())
}

/// vcu_apps1_uv_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_apps1_uv_threshold(&self) -> u16 {
    self.vcu_apps1_uv_threshold_raw()
}

/// Get raw value of vcu_apps1_uv_threshold
///
/// - Start bit: 24
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_apps1_uv_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[24..40].load_le::<u16>();
    
    signal
}

/// Set value of vcu_apps1_uv_threshold
#[inline(always)]
pub fn set_vcu_apps1_uv_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[24..40].store_le(value);
    Ok(())
}

/// vcu_brake_active_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_brake_active_threshold(&self) -> u16 {
    self.vcu_brake_active_threshold_raw()
}

/// Get raw value of vcu_brake_active_threshold
///
/// - Start bit: 8
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_brake_active_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[8..24].load_le::<u16>();
    
    signal
}

/// Set value of vcu_brake_active_threshold
#[inline(always)]
pub fn set_vcu_brake_active_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[8..24].store_le(value);
    Ok(())
}

}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct VcuPedalThresholdSettingsVcuPedalsSettingsMuxM1 { raw: [u8; 7] }

impl VcuPedalThresholdSettingsVcuPedalsSettingsMuxM1 {
pub fn new() -> Self { Self { raw: [0u8; 7] } }
/// vcu_apps2_uv_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_apps2_uv_threshold(&self) -> u16 {
    self.vcu_apps2_uv_threshold_raw()
}

/// Get raw value of vcu_apps2_uv_threshold
///
/// - Start bit: 40
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_apps2_uv_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[40..56].load_le::<u16>();
    
    signal
}

/// Set value of vcu_apps2_uv_threshold
#[inline(always)]
pub fn set_vcu_apps2_uv_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[40..56].store_le(value);
    Ok(())
}

/// vcu_apps1_ov_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_apps1_ov_threshold(&self) -> u16 {
    self.vcu_apps1_ov_threshold_raw()
}

/// Get raw value of vcu_apps1_ov_threshold
///
/// - Start bit: 24
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_apps1_ov_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[24..40].load_le::<u16>();
    
    signal
}

/// Set value of vcu_apps1_ov_threshold
#[inline(always)]
pub fn set_vcu_apps1_ov_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[24..40].store_le(value);
    Ok(())
}

/// vcu_apps1_end_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_apps1_end_threshold(&self) -> u16 {
    self.vcu_apps1_end_threshold_raw()
}

/// Get raw value of vcu_apps1_end_threshold
///
/// - Start bit: 8
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_apps1_end_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[8..24].load_le::<u16>();
    
    signal
}

/// Set value of vcu_apps1_end_threshold
#[inline(always)]
pub fn set_vcu_apps1_end_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[8..24].store_le(value);
    Ok(())
}

}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct VcuPedalThresholdSettingsVcuPedalsSettingsMuxM2 { raw: [u8; 7] }

impl VcuPedalThresholdSettingsVcuPedalsSettingsMuxM2 {
pub fn new() -> Self { Self { raw: [0u8; 7] } }
/// vcu_apps2_ov_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_apps2_ov_threshold(&self) -> u16 {
    self.vcu_apps2_ov_threshold_raw()
}

/// Get raw value of vcu_apps2_ov_threshold
///
/// - Start bit: 40
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_apps2_ov_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[40..56].load_le::<u16>();
    
    signal
}

/// Set value of vcu_apps2_ov_threshold
#[inline(always)]
pub fn set_vcu_apps2_ov_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[40..56].store_le(value);
    Ok(())
}

/// vcu_apps2_end_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_apps2_end_threshold(&self) -> u16 {
    self.vcu_apps2_end_threshold_raw()
}

/// Get raw value of vcu_apps2_end_threshold
///
/// - Start bit: 24
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_apps2_end_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[24..40].load_le::<u16>();
    
    signal
}

/// Set value of vcu_apps2_end_threshold
#[inline(always)]
pub fn set_vcu_apps2_end_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[24..40].store_le(value);
    Ok(())
}

/// vcu_apps2_start_threshold
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn vcu_apps2_start_threshold(&self) -> u16 {
    self.vcu_apps2_start_threshold_raw()
}

/// Get raw value of vcu_apps2_start_threshold
///
/// - Start bit: 8
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn vcu_apps2_start_threshold_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[8..24].load_le::<u16>();
    
    signal
}

/// Set value of vcu_apps2_start_threshold
#[inline(always)]
pub fn set_vcu_apps2_start_threshold(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 199 });
    }
    self.raw.view_bits_mut::<Lsb0>()[8..24].store_le(value);
    Ok(())
}

}


/// vcu_board_data
///
/// - ID: 200 (0xc8)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// information on the running vcu firmware and system on-timer
#[derive(Clone, Copy)]
pub struct VcuBoardData {
    raw: [u8; 8],
}

impl VcuBoardData {
    pub const MESSAGE_ID: u32 = 200;
    
    pub const BOARD_ON_TIME_SECONDS_MIN: u16 = 0_u16;
    pub const BOARD_ON_TIME_SECONDS_MAX: u16 = 65535_u16;
    pub const FIRMWARE_VERSION_MIN: u32 = 0_u32;
    pub const FIRMWARE_VERSION_MAX: u32 = 4294967295_u32;
    
    /// Construct new vcu_board_data from values
    pub fn new(firmware_is_dirty: bool, firmware_on_main: bool, board_on_time_seconds: u16, firmware_version: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_firmware_is_dirty(firmware_is_dirty)?;
        res.set_firmware_on_main(firmware_on_main)?;
        res.set_board_on_time_seconds(board_on_time_seconds)?;
        res.set_firmware_version(firmware_version)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// firmware_is_dirty
    ///
    /// if the compiled firmware had uncommitted changes (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_is_dirty(&self) -> bool {
        self.firmware_is_dirty_raw()
    }
    
    /// Get raw value of firmware_is_dirty
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_is_dirty_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_is_dirty
    #[inline(always)]
    pub fn set_firmware_is_dirty(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// firmware_on_main
    ///
    /// if the compiled firmware was from main branch (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_on_main(&self) -> bool {
        self.firmware_on_main_raw()
    }
    
    /// Get raw value of firmware_on_main
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_on_main_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_on_main
    #[inline(always)]
    pub fn set_firmware_on_main(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// board_on_time_seconds
    ///
    /// time in seconds which the board has been powered on and running
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_on_time_seconds(&self) -> u16 {
        self.board_on_time_seconds_raw()
    }
    
    /// Get raw value of board_on_time_seconds
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_on_time_seconds_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of board_on_time_seconds
    #[inline(always)]
    pub fn set_board_on_time_seconds(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 200 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// firmware_version
    ///
    /// short hash of the firmware
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version_raw()
    }
    
    /// Get raw value of firmware_version
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_version_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of firmware_version
    #[inline(always)]
    pub fn set_firmware_version(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 200 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuBoardData {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuBoardData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuBoardData")
                .field("firmware_is_dirty", &self.firmware_is_dirty())
                .field("firmware_on_main", &self.firmware_on_main())
                .field("board_on_time_seconds", &self.board_on_time_seconds())
                .field("firmware_version", &self.firmware_version())
            .finish()
        } else {
            f.debug_tuple("VcuBoardData").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuBoardData {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let firmware_is_dirty = u.int_in_range(0..=1)? == 1;
        let firmware_on_main = u.int_in_range(0..=1)? == 1;
        let board_on_time_seconds = u.int_in_range(0..=65535)?;
        let firmware_version = u.int_in_range(0..=4294967295)?;
        VcuBoardData::new(firmware_is_dirty,firmware_on_main,board_on_time_seconds,firmware_version).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_board_readings_one
///
/// - ID: 201 (0xc9)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// VCU PCB health readings
#[derive(Clone, Copy)]
pub struct VcuBoardReadingsOne {
    raw: [u8; 8],
}

impl VcuBoardReadingsOne {
    pub const MESSAGE_ID: u32 = 201;
    
    pub const VCU_5V_VOLTAGE_MIN: u16 = 0_u16;
    pub const VCU_5V_VOLTAGE_MAX: u16 = 65535_u16;
    pub const VCU_BSPD_VOLTAGE_MIN: u16 = 0_u16;
    pub const VCU_BSPD_VOLTAGE_MAX: u16 = 65535_u16;
    pub const VCU_GLV_VOLTAGE_MIN: u16 = 0_u16;
    pub const VCU_GLV_VOLTAGE_MAX: u16 = 65535_u16;
    pub const VCU_GLV_CURRENT_MIN: u16 = 0_u16;
    pub const VCU_GLV_CURRENT_MAX: u16 = 65535_u16;
    
    /// Construct new vcu_board_readings_one from values
    pub fn new(vcu_5v_voltage: u16, vcu_bspd_voltage: u16, vcu_glv_voltage: u16, vcu_glv_current: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_5v_voltage(vcu_5v_voltage)?;
        res.set_vcu_bspd_voltage(vcu_bspd_voltage)?;
        res.set_vcu_glv_voltage(vcu_glv_voltage)?;
        res.set_vcu_glv_current(vcu_glv_current)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// VCU_5V_VOLTAGE
    ///
    /// VCU pcb voltage sense of the 5v rail
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "bits"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_5v_voltage(&self) -> u16 {
        self.vcu_5v_voltage_raw()
    }
    
    /// Get raw value of VCU_5V_VOLTAGE
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_5v_voltage_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_5V_VOLTAGE
    #[inline(always)]
    pub fn set_vcu_5v_voltage(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 201 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// VCU_BSPD_VOLTAGE
    ///
    /// VCU pcb voltage sense of the BSPD signal to close relay
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "bits"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_bspd_voltage(&self) -> u16 {
        self.vcu_bspd_voltage_raw()
    }
    
    /// Get raw value of VCU_BSPD_VOLTAGE
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_bspd_voltage_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_BSPD_VOLTAGE
    #[inline(always)]
    pub fn set_vcu_bspd_voltage(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 201 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// VCU_GLV_VOLTAGE
    ///
    /// VCU pcb voltage sense of glv rail
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "bits"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_glv_voltage(&self) -> u16 {
        self.vcu_glv_voltage_raw()
    }
    
    /// Get raw value of VCU_GLV_VOLTAGE
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_glv_voltage_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_GLV_VOLTAGE
    #[inline(always)]
    pub fn set_vcu_glv_voltage(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 201 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// VCU_GLV_CURRENT
    ///
    /// VCU pcb current sense of the glv rail
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "bits"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_glv_current(&self) -> u16 {
        self.vcu_glv_current_raw()
    }
    
    /// Get raw value of VCU_GLV_CURRENT
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_glv_current_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_GLV_CURRENT
    #[inline(always)]
    pub fn set_vcu_glv_current(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 201 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuBoardReadingsOne {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuBoardReadingsOne {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuBoardReadingsOne")
                .field("vcu_5v_voltage", &self.vcu_5v_voltage())
                .field("vcu_bspd_voltage", &self.vcu_bspd_voltage())
                .field("vcu_glv_voltage", &self.vcu_glv_voltage())
                .field("vcu_glv_current", &self.vcu_glv_current())
            .finish()
        } else {
            f.debug_tuple("VcuBoardReadingsOne").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuBoardReadingsOne {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_5v_voltage = u.int_in_range(0..=65535)?;
        let vcu_bspd_voltage = u.int_in_range(0..=65535)?;
        let vcu_glv_voltage = u.int_in_range(0..=65535)?;
        let vcu_glv_current = u.int_in_range(0..=65535)?;
        VcuBoardReadingsOne::new(vcu_5v_voltage,vcu_bspd_voltage,vcu_glv_voltage,vcu_glv_current).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_board_readings_two
///
/// - ID: 202 (0xca)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// VCU PCB health readings
#[derive(Clone, Copy)]
pub struct VcuBoardReadingsTwo {
    raw: [u8; 8],
}

impl VcuBoardReadingsTwo {
    pub const MESSAGE_ID: u32 = 202;
    
    pub const VCU_AIN10_VOLTAGE_MIN: u16 = 0_u16;
    pub const VCU_AIN10_VOLTAGE_MAX: u16 = 65535_u16;
    pub const VCU_AIN9_VOLTAGE_MIN: u16 = 0_u16;
    pub const VCU_AIN9_VOLTAGE_MAX: u16 = 65535_u16;
    pub const VCU_SDC_CURRENT_MIN: u16 = 0_u16;
    pub const VCU_SDC_CURRENT_MAX: u16 = 65535_u16;
    pub const VCU_SDC_VOLTAGE_MIN: u16 = 0_u16;
    pub const VCU_SDC_VOLTAGE_MAX: u16 = 65535_u16;
    
    /// Construct new vcu_board_readings_two from values
    pub fn new(vcu_ain10_voltage: u16, vcu_ain9_voltage: u16, vcu_sdc_current: u16, vcu_sdc_voltage: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_ain10_voltage(vcu_ain10_voltage)?;
        res.set_vcu_ain9_voltage(vcu_ain9_voltage)?;
        res.set_vcu_sdc_current(vcu_sdc_current)?;
        res.set_vcu_sdc_voltage(vcu_sdc_voltage)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// VCU_AIN10_VOLTAGE
    ///
    /// VCU pcb non specific analog channel
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "bits"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_ain10_voltage(&self) -> u16 {
        self.vcu_ain10_voltage_raw()
    }
    
    /// Get raw value of VCU_AIN10_VOLTAGE
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_ain10_voltage_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_AIN10_VOLTAGE
    #[inline(always)]
    pub fn set_vcu_ain10_voltage(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 202 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// VCU_AIN9_VOLTAGE
    ///
    /// VCU pcb non specific analog channel
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "bits"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_ain9_voltage(&self) -> u16 {
        self.vcu_ain9_voltage_raw()
    }
    
    /// Get raw value of VCU_AIN9_VOLTAGE
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_ain9_voltage_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_AIN9_VOLTAGE
    #[inline(always)]
    pub fn set_vcu_ain9_voltage(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 202 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// VCU_SDC_CURRENT
    ///
    /// VCU PCB current sense of shutdown circuit
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "bits"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_sdc_current(&self) -> u16 {
        self.vcu_sdc_current_raw()
    }
    
    /// Get raw value of VCU_SDC_CURRENT
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_sdc_current_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_SDC_CURRENT
    #[inline(always)]
    pub fn set_vcu_sdc_current(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 202 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// VCU_SDC_VOLTAGE
    ///
    /// VCU pcb voltage sense of the shutdown circuit input
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "bits"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_sdc_voltage(&self) -> u16 {
        self.vcu_sdc_voltage_raw()
    }
    
    /// Get raw value of VCU_SDC_VOLTAGE
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_sdc_voltage_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_SDC_VOLTAGE
    #[inline(always)]
    pub fn set_vcu_sdc_voltage(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 202 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuBoardReadingsTwo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuBoardReadingsTwo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuBoardReadingsTwo")
                .field("vcu_ain10_voltage", &self.vcu_ain10_voltage())
                .field("vcu_ain9_voltage", &self.vcu_ain9_voltage())
                .field("vcu_sdc_current", &self.vcu_sdc_current())
                .field("vcu_sdc_voltage", &self.vcu_sdc_voltage())
            .finish()
        } else {
            f.debug_tuple("VcuBoardReadingsTwo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuBoardReadingsTwo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_ain10_voltage = u.int_in_range(0..=65535)?;
        let vcu_ain9_voltage = u.int_in_range(0..=65535)?;
        let vcu_sdc_current = u.int_in_range(0..=65535)?;
        let vcu_sdc_voltage = u.int_in_range(0..=65535)?;
        VcuBoardReadingsTwo::new(vcu_ain10_voltage,vcu_ain9_voltage,vcu_sdc_current,vcu_sdc_voltage).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_pedals_travel
///
/// - ID: 204 (0xcc)
/// - Size: 6 bytes
/// - Transmitter: vcu
///
/// the calculated pedal travels
#[derive(Clone, Copy)]
pub struct VcuPedalsTravel {
    raw: [u8; 6],
}

impl VcuPedalsTravel {
    pub const MESSAGE_ID: u32 = 204;
    
    pub const VCU_BSE1_TRAVEL_MIN: f32 = 0_f32;
    pub const VCU_BSE1_TRAVEL_MAX: f32 = 65535_f32;
    pub const VCU_APPS2_TRAVEL_MIN: f32 = 0_f32;
    pub const VCU_APPS2_TRAVEL_MAX: f32 = 65535_f32;
    pub const VCU_APPS1_TRAVEL_MIN: f32 = 0_f32;
    pub const VCU_APPS1_TRAVEL_MAX: f32 = 65535_f32;
    
    /// Construct new vcu_pedals_travel from values
    pub fn new(vcu_bse1_travel: f32, vcu_apps2_travel: f32, vcu_apps1_travel: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 6] };
        res.set_vcu_bse1_travel(vcu_bse1_travel)?;
        res.set_vcu_apps2_travel(vcu_apps2_travel)?;
        res.set_vcu_apps1_travel(vcu_apps1_travel)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 6] {
        &self.raw
    }
    
    /// vcu_bse1_travel
    ///
    /// 0 to 100% - a negative value = implaus
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_bse1_travel(&self) -> f32 {
        self.vcu_bse1_travel_raw()
    }
    
    /// Get raw value of vcu_bse1_travel
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_bse1_travel_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_bse1_travel
    #[inline(always)]
    pub fn set_vcu_bse1_travel(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 204 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// vcu_apps2_travel
    ///
    /// 0 to 100% - a negative value = implaus
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_apps2_travel(&self) -> f32 {
        self.vcu_apps2_travel_raw()
    }
    
    /// Get raw value of vcu_apps2_travel
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_apps2_travel_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_apps2_travel
    #[inline(always)]
    pub fn set_vcu_apps2_travel(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 204 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// vcu_apps1_travel
    ///
    /// 0 to 100% - a negative value = implaus
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_apps1_travel(&self) -> f32 {
        self.vcu_apps1_travel_raw()
    }
    
    /// Get raw value of vcu_apps1_travel
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_apps1_travel_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_apps1_travel
    #[inline(always)]
    pub fn set_vcu_apps1_travel(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 204 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuPedalsTravel {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 6 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 6];
        raw.copy_from_slice(&payload[..6]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuPedalsTravel {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuPedalsTravel")
                .field("vcu_bse1_travel", &self.vcu_bse1_travel())
                .field("vcu_apps2_travel", &self.vcu_apps2_travel())
                .field("vcu_apps1_travel", &self.vcu_apps1_travel())
            .finish()
        } else {
            f.debug_tuple("VcuPedalsTravel").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuPedalsTravel {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_bse1_travel = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_apps2_travel = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_apps1_travel = u.float_in_range(0_f32..=65535_f32)?;
        VcuPedalsTravel::new(vcu_bse1_travel,vcu_apps2_travel,vcu_apps1_travel).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_launchcontrol_countdown
///
/// - ID: 205 (0xcd)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// Countdown until the VCU will start launch when in launch mode
#[derive(Clone, Copy)]
pub struct VcuLaunchcontrolCountdown {
    raw: [u8; 8],
}

impl VcuLaunchcontrolCountdown {
    pub const MESSAGE_ID: u32 = 205;
    
    pub const VCU_RELEASE_DELAY_MIN: u32 = 0_u32;
    pub const VCU_RELEASE_DELAY_MAX: u32 = 4294967295_u32;
    pub const VCU_LAUNCHCONTROL_RELEASE_COUNTD_MIN: u32 = 0_u32;
    pub const VCU_LAUNCHCONTROL_RELEASE_COUNTD_MAX: u32 = 4294967295_u32;
    
    /// Construct new vcu_launchcontrol_countdown from values
    pub fn new(vcu_release_delay: u32, vcu_launchcontrol_release_countd: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_release_delay(vcu_release_delay)?;
        res.set_vcu_launchcontrol_release_countd(vcu_launchcontrol_release_countd)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// vcu_release_delay
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: "milliseconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_release_delay(&self) -> u32 {
        self.vcu_release_delay_raw()
    }
    
    /// Get raw value of vcu_release_delay
    ///
    /// - Start bit: 32
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_release_delay_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..64].load_le::<u32>();
        
        signal
    }
    
    /// Set value of vcu_release_delay
    #[inline(always)]
    pub fn set_vcu_release_delay(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 205 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..64].store_le(value);
        Ok(())
    }
    
    /// vcu_launchcontrol_release_countd
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: "milliseconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_launchcontrol_release_countd(&self) -> u32 {
        self.vcu_launchcontrol_release_countd_raw()
    }
    
    /// Get raw value of vcu_launchcontrol_release_countd
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_launchcontrol_release_countd_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of vcu_launchcontrol_release_countd
    #[inline(always)]
    pub fn set_vcu_launchcontrol_release_countd(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 205 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuLaunchcontrolCountdown {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuLaunchcontrolCountdown {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuLaunchcontrolCountdown")
                .field("vcu_release_delay", &self.vcu_release_delay())
                .field("vcu_launchcontrol_release_countd", &self.vcu_launchcontrol_release_countd())
            .finish()
        } else {
            f.debug_tuple("VcuLaunchcontrolCountdown").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuLaunchcontrolCountdown {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_release_delay = u.int_in_range(0..=4294967295)?;
        let vcu_launchcontrol_release_countd = u.int_in_range(0..=4294967295)?;
        VcuLaunchcontrolCountdown::new(vcu_release_delay,vcu_launchcontrol_release_countd).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_distance_tracker_motor
///
/// - ID: 206 (0xce)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// distance tracking data using motor as speed
#[derive(Clone, Copy)]
pub struct VcuDistanceTrackerMotor {
    raw: [u8; 8],
}

impl VcuDistanceTrackerMotor {
    pub const MESSAGE_ID: u32 = 206;
    
    pub const VCU_MOTOR_EFFICIENCY_KMKWH_MIN: f32 = 0_f32;
    pub const VCU_MOTOR_EFFICIENCY_KMKWH_MAX: f32 = 65535_f32;
    pub const VCU_MOTOR_DISTANCE_METERS_MIN: u16 = 0_u16;
    pub const VCU_MOTOR_DISTANCE_METERS_MAX: u16 = 65535_u16;
    pub const VCU_MOTOR_EFFICIENCY_INSTANTANEO_MIN: f32 = 0_f32;
    pub const VCU_MOTOR_EFFICIENCY_INSTANTANEO_MAX: f32 = 65535_f32;
    pub const VCU_MOTOR_ENERGY_WH_MIN: f32 = 0_f32;
    pub const VCU_MOTOR_ENERGY_WH_MAX: f32 = 65535_f32;
    
    /// Construct new vcu_distance_tracker_motor from values
    pub fn new(vcu_motor_efficiency_kmkwh: f32, vcu_motor_distance_meters: u16, vcu_motor_efficiency_instantaneo: f32, vcu_motor_energy_wh: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_motor_efficiency_kmkwh(vcu_motor_efficiency_kmkwh)?;
        res.set_vcu_motor_distance_meters(vcu_motor_distance_meters)?;
        res.set_vcu_motor_efficiency_instantaneo(vcu_motor_efficiency_instantaneo)?;
        res.set_vcu_motor_energy_wh(vcu_motor_energy_wh)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// vcu_motor_efficiency_kmkwh
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_motor_efficiency_kmkwh(&self) -> f32 {
        self.vcu_motor_efficiency_kmkwh_raw()
    }
    
    /// Get raw value of vcu_motor_efficiency_kmkwh
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_motor_efficiency_kmkwh_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let factor = 0.001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_motor_efficiency_kmkwh
    #[inline(always)]
    pub fn set_vcu_motor_efficiency_kmkwh(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 206 });
        }
        let factor = 0.001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// vcu_motor_distance_meters
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_motor_distance_meters(&self) -> u16 {
        self.vcu_motor_distance_meters_raw()
    }
    
    /// Get raw value of vcu_motor_distance_meters
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_motor_distance_meters_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of vcu_motor_distance_meters
    #[inline(always)]
    pub fn set_vcu_motor_distance_meters(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 206 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// vcu_motor_efficiency_instantaneo
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_motor_efficiency_instantaneo(&self) -> f32 {
        self.vcu_motor_efficiency_instantaneo_raw()
    }
    
    /// Get raw value of vcu_motor_efficiency_instantaneo
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_motor_efficiency_instantaneo_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let factor = 0.001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_motor_efficiency_instantaneo
    #[inline(always)]
    pub fn set_vcu_motor_efficiency_instantaneo(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 206 });
        }
        let factor = 0.001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// vcu_motor_energy_wh
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "watt-hours"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_motor_energy_wh(&self) -> f32 {
        self.vcu_motor_energy_wh_raw()
    }
    
    /// Get raw value of vcu_motor_energy_wh
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_motor_energy_wh_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_motor_energy_wh
    #[inline(always)]
    pub fn set_vcu_motor_energy_wh(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 206 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuDistanceTrackerMotor {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuDistanceTrackerMotor {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuDistanceTrackerMotor")
                .field("vcu_motor_efficiency_kmkwh", &self.vcu_motor_efficiency_kmkwh())
                .field("vcu_motor_distance_meters", &self.vcu_motor_distance_meters())
                .field("vcu_motor_efficiency_instantaneo", &self.vcu_motor_efficiency_instantaneo())
                .field("vcu_motor_energy_wh", &self.vcu_motor_energy_wh())
            .finish()
        } else {
            f.debug_tuple("VcuDistanceTrackerMotor").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuDistanceTrackerMotor {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_motor_efficiency_kmkwh = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_motor_distance_meters = u.int_in_range(0..=65535)?;
        let vcu_motor_efficiency_instantaneo = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_motor_energy_wh = u.float_in_range(0_f32..=65535_f32)?;
        VcuDistanceTrackerMotor::new(vcu_motor_efficiency_kmkwh,vcu_motor_distance_meters,vcu_motor_efficiency_instantaneo,vcu_motor_energy_wh).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_distance_tracker_wheelspeed
///
/// - ID: 207 (0xcf)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// distance tracking data using wheelspeed as speed
#[derive(Clone, Copy)]
pub struct VcuDistanceTrackerWheelspeed {
    raw: [u8; 8],
}

impl VcuDistanceTrackerWheelspeed {
    pub const MESSAGE_ID: u32 = 207;
    
    pub const VCU_WHEELSPEED_EFFICIENCY_KMKWH_MIN: f32 = 0_f32;
    pub const VCU_WHEELSPEED_EFFICIENCY_KMKWH_MAX: f32 = 65535_f32;
    pub const VCU_WHEELSPEED_DISTANCE_METERS_MIN: u16 = 0_u16;
    pub const VCU_WHEELSPEED_DISTANCE_METERS_MAX: u16 = 65535_u16;
    pub const VCU_WHEELSPEED_EFFICIENCY_INSTAN_MIN: f32 = 0_f32;
    pub const VCU_WHEELSPEED_EFFICIENCY_INSTAN_MAX: f32 = 65535_f32;
    pub const VCU_WHEELSPEED_ENERGY_WH_MIN: f32 = 0_f32;
    pub const VCU_WHEELSPEED_ENERGY_WH_MAX: f32 = 65535_f32;
    
    /// Construct new vcu_distance_tracker_wheelspeed from values
    pub fn new(vcu_wheelspeed_efficiency_kmkwh: f32, vcu_wheelspeed_distance_meters: u16, vcu_wheelspeed_efficiency_instan: f32, vcu_wheelspeed_energy_wh: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_wheelspeed_efficiency_kmkwh(vcu_wheelspeed_efficiency_kmkwh)?;
        res.set_vcu_wheelspeed_distance_meters(vcu_wheelspeed_distance_meters)?;
        res.set_vcu_wheelspeed_efficiency_instan(vcu_wheelspeed_efficiency_instan)?;
        res.set_vcu_wheelspeed_energy_wh(vcu_wheelspeed_energy_wh)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// vcu_wheelspeed_efficiency_kmkwh
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_wheelspeed_efficiency_kmkwh(&self) -> f32 {
        self.vcu_wheelspeed_efficiency_kmkwh_raw()
    }
    
    /// Get raw value of vcu_wheelspeed_efficiency_kmkwh
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_wheelspeed_efficiency_kmkwh_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let factor = 0.001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_wheelspeed_efficiency_kmkwh
    #[inline(always)]
    pub fn set_vcu_wheelspeed_efficiency_kmkwh(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 207 });
        }
        let factor = 0.001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// vcu_wheelspeed_distance_meters
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_wheelspeed_distance_meters(&self) -> u16 {
        self.vcu_wheelspeed_distance_meters_raw()
    }
    
    /// Get raw value of vcu_wheelspeed_distance_meters
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_wheelspeed_distance_meters_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of vcu_wheelspeed_distance_meters
    #[inline(always)]
    pub fn set_vcu_wheelspeed_distance_meters(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 207 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// vcu_wheelspeed_efficiency_instan
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_wheelspeed_efficiency_instan(&self) -> f32 {
        self.vcu_wheelspeed_efficiency_instan_raw()
    }
    
    /// Get raw value of vcu_wheelspeed_efficiency_instan
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_wheelspeed_efficiency_instan_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let factor = 0.001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_wheelspeed_efficiency_instan
    #[inline(always)]
    pub fn set_vcu_wheelspeed_efficiency_instan(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 207 });
        }
        let factor = 0.001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// vcu_wheelspeed_energy_wh
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "watt-hours"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_wheelspeed_energy_wh(&self) -> f32 {
        self.vcu_wheelspeed_energy_wh_raw()
    }
    
    /// Get raw value of vcu_wheelspeed_energy_wh
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_wheelspeed_energy_wh_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_wheelspeed_energy_wh
    #[inline(always)]
    pub fn set_vcu_wheelspeed_energy_wh(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 207 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuDistanceTrackerWheelspeed {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuDistanceTrackerWheelspeed {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuDistanceTrackerWheelspeed")
                .field("vcu_wheelspeed_efficiency_kmkwh", &self.vcu_wheelspeed_efficiency_kmkwh())
                .field("vcu_wheelspeed_distance_meters", &self.vcu_wheelspeed_distance_meters())
                .field("vcu_wheelspeed_efficiency_instan", &self.vcu_wheelspeed_efficiency_instan())
                .field("vcu_wheelspeed_energy_wh", &self.vcu_wheelspeed_energy_wh())
            .finish()
        } else {
            f.debug_tuple("VcuDistanceTrackerWheelspeed").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuDistanceTrackerWheelspeed {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_wheelspeed_efficiency_kmkwh = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_wheelspeed_distance_meters = u.int_in_range(0..=65535)?;
        let vcu_wheelspeed_efficiency_instan = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_wheelspeed_energy_wh = u.float_in_range(0_f32..=65535_f32)?;
        VcuDistanceTrackerWheelspeed::new(vcu_wheelspeed_efficiency_kmkwh,vcu_wheelspeed_distance_meters,vcu_wheelspeed_efficiency_instan,vcu_wheelspeed_energy_wh).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_lifetime_distance_and_ontime
///
/// - ID: 208 (0xd0)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// The total ontime of the vcu and distance
#[derive(Clone, Copy)]
pub struct VcuLifetimeDistanceAndOntime {
    raw: [u8; 8],
}

impl VcuLifetimeDistanceAndOntime {
    pub const MESSAGE_ID: u32 = 208;
    
    pub const VCU_LIFETIME_DISTANCE_MIN: u32 = 0_u32;
    pub const VCU_LIFETIME_DISTANCE_MAX: u32 = 4294967295_u32;
    pub const VCU_LIFETIME_ONTIME_MIN: u32 = 0_u32;
    pub const VCU_LIFETIME_ONTIME_MAX: u32 = 4294967295_u32;
    
    /// Construct new vcu_lifetime_distance_and_ontime from values
    pub fn new(vcu_lifetime_distance: u32, vcu_lifetime_ontime: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_lifetime_distance(vcu_lifetime_distance)?;
        res.set_vcu_lifetime_ontime(vcu_lifetime_ontime)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// vcu_lifetime_distance
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: "meters"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_lifetime_distance(&self) -> u32 {
        self.vcu_lifetime_distance_raw()
    }
    
    /// Get raw value of vcu_lifetime_distance
    ///
    /// - Start bit: 32
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_lifetime_distance_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..64].load_le::<u32>();
        
        signal
    }
    
    /// Set value of vcu_lifetime_distance
    #[inline(always)]
    pub fn set_vcu_lifetime_distance(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 208 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..64].store_le(value);
        Ok(())
    }
    
    /// vcu_lifetime_ontime
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_lifetime_ontime(&self) -> u32 {
        self.vcu_lifetime_ontime_raw()
    }
    
    /// Get raw value of vcu_lifetime_ontime
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_lifetime_ontime_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of vcu_lifetime_ontime
    #[inline(always)]
    pub fn set_vcu_lifetime_ontime(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 208 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuLifetimeDistanceAndOntime {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuLifetimeDistanceAndOntime {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuLifetimeDistanceAndOntime")
                .field("vcu_lifetime_distance", &self.vcu_lifetime_distance())
                .field("vcu_lifetime_ontime", &self.vcu_lifetime_ontime())
            .finish()
        } else {
            f.debug_tuple("VcuLifetimeDistanceAndOntime").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuLifetimeDistanceAndOntime {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_lifetime_distance = u.int_in_range(0..=4294967295)?;
        let vcu_lifetime_ontime = u.int_in_range(0..=4294967295)?;
        VcuLifetimeDistanceAndOntime::new(vcu_lifetime_distance,vcu_lifetime_ontime).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_distance_tracker_vectornav
///
/// - ID: 210 (0xd2)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// distance tracking data using vectornav velocity measurement as speed
#[derive(Clone, Copy)]
pub struct VcuDistanceTrackerVectornav {
    raw: [u8; 8],
}

impl VcuDistanceTrackerVectornav {
    pub const MESSAGE_ID: u32 = 210;
    
    pub const VCU_VECTORNAV_EFFICIENCY_KMKWH_MIN: f32 = 0_f32;
    pub const VCU_VECTORNAV_EFFICIENCY_KMKWH_MAX: f32 = 65535_f32;
    pub const VCU_VECTORNAV_DISTANCE_METERS_MIN: u16 = 0_u16;
    pub const VCU_VECTORNAV_DISTANCE_METERS_MAX: u16 = 65535_u16;
    pub const VCU_VECTORNAV_EFFICIENCY_INSTANT_MIN: f32 = 0_f32;
    pub const VCU_VECTORNAV_EFFICIENCY_INSTANT_MAX: f32 = 65535_f32;
    pub const VCU_VECTORNAV_ENERGY_WH_MIN: f32 = 0_f32;
    pub const VCU_VECTORNAV_ENERGY_WH_MAX: f32 = 65535_f32;
    
    /// Construct new vcu_distance_tracker_vectornav from values
    pub fn new(vcu_vectornav_efficiency_kmkwh: f32, vcu_vectornav_distance_meters: u16, vcu_vectornav_efficiency_instant: f32, vcu_vectornav_energy_wh: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_vectornav_efficiency_kmkwh(vcu_vectornav_efficiency_kmkwh)?;
        res.set_vcu_vectornav_distance_meters(vcu_vectornav_distance_meters)?;
        res.set_vcu_vectornav_efficiency_instant(vcu_vectornav_efficiency_instant)?;
        res.set_vcu_vectornav_energy_wh(vcu_vectornav_energy_wh)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// vcu_vectornav_efficiency_kmkwh
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_vectornav_efficiency_kmkwh(&self) -> f32 {
        self.vcu_vectornav_efficiency_kmkwh_raw()
    }
    
    /// Get raw value of vcu_vectornav_efficiency_kmkwh
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_vectornav_efficiency_kmkwh_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let factor = 0.001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_vectornav_efficiency_kmkwh
    #[inline(always)]
    pub fn set_vcu_vectornav_efficiency_kmkwh(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 210 });
        }
        let factor = 0.001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// vcu_vectornav_distance_meters
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_vectornav_distance_meters(&self) -> u16 {
        self.vcu_vectornav_distance_meters_raw()
    }
    
    /// Get raw value of vcu_vectornav_distance_meters
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_vectornav_distance_meters_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of vcu_vectornav_distance_meters
    #[inline(always)]
    pub fn set_vcu_vectornav_distance_meters(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 210 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// vcu_vectornav_efficiency_instant
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_vectornav_efficiency_instant(&self) -> f32 {
        self.vcu_vectornav_efficiency_instant_raw()
    }
    
    /// Get raw value of vcu_vectornav_efficiency_instant
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_vectornav_efficiency_instant_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let factor = 0.001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_vectornav_efficiency_instant
    #[inline(always)]
    pub fn set_vcu_vectornav_efficiency_instant(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 210 });
        }
        let factor = 0.001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// vcu_vectornav_energy_wh
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "watt-hours"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_vectornav_energy_wh(&self) -> f32 {
        self.vcu_vectornav_energy_wh_raw()
    }
    
    /// Get raw value of vcu_vectornav_energy_wh
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_vectornav_energy_wh_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_vectornav_energy_wh
    #[inline(always)]
    pub fn set_vcu_vectornav_energy_wh(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 210 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuDistanceTrackerVectornav {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuDistanceTrackerVectornav {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuDistanceTrackerVectornav")
                .field("vcu_vectornav_efficiency_kmkwh", &self.vcu_vectornav_efficiency_kmkwh())
                .field("vcu_vectornav_distance_meters", &self.vcu_vectornav_distance_meters())
                .field("vcu_vectornav_efficiency_instant", &self.vcu_vectornav_efficiency_instant())
                .field("vcu_vectornav_energy_wh", &self.vcu_vectornav_energy_wh())
            .finish()
        } else {
            f.debug_tuple("VcuDistanceTrackerVectornav").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuDistanceTrackerVectornav {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_vectornav_efficiency_kmkwh = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_vectornav_distance_meters = u.int_in_range(0..=65535)?;
        let vcu_vectornav_efficiency_instant = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_vectornav_energy_wh = u.float_in_range(0_f32..=65535_f32)?;
        VcuDistanceTrackerVectornav::new(vcu_vectornav_efficiency_kmkwh,vcu_vectornav_distance_meters,vcu_vectornav_efficiency_instant,vcu_vectornav_energy_wh).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_coulomb_counters
///
/// - ID: 211 (0xd3)
/// - Size: 8 bytes
/// - Transmitter: vcu
///
/// vcu amp-hour counters
#[derive(Clone, Copy)]
pub struct VcuCoulombCounters {
    raw: [u8; 8],
}

impl VcuCoulombCounters {
    pub const MESSAGE_ID: u32 = 211;
    
    pub const VCU_VNAV_AH_MIN: f32 = 0_f32;
    pub const VCU_VNAV_AH_MAX: f32 = 65535_f32;
    pub const VCU_MOTOR_AH_MIN: f32 = 0_f32;
    pub const VCU_MOTOR_AH_MAX: f32 = 65535_f32;
    pub const VCU_WSFL_AH_MIN: f32 = 0_f32;
    pub const VCU_WSFL_AH_MAX: f32 = 65535_f32;
    
    /// Construct new vcu_coulomb_counters from values
    pub fn new(vcu_vnav_ah: f32, vcu_motor_ah: f32, vcu_wsfl_ah: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_vnav_ah(vcu_vnav_ah)?;
        res.set_vcu_motor_ah(vcu_motor_ah)?;
        res.set_vcu_wsfl_ah(vcu_wsfl_ah)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// vcu_vnav_ah
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_vnav_ah(&self) -> f32 {
        self.vcu_vnav_ah_raw()
    }
    
    /// Get raw value of vcu_vnav_ah
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_vnav_ah_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_vnav_ah
    #[inline(always)]
    pub fn set_vcu_vnav_ah(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 211 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// vcu_motor_ah
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_motor_ah(&self) -> f32 {
        self.vcu_motor_ah_raw()
    }
    
    /// Get raw value of vcu_motor_ah
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_motor_ah_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_motor_ah
    #[inline(always)]
    pub fn set_vcu_motor_ah(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 211 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// vcu_wsfl_ah
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_wsfl_ah(&self) -> f32 {
        self.vcu_wsfl_ah_raw()
    }
    
    /// Get raw value of vcu_wsfl_ah
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_wsfl_ah_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_wsfl_ah
    #[inline(always)]
    pub fn set_vcu_wsfl_ah(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 211 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuCoulombCounters {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuCoulombCounters {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuCoulombCounters")
                .field("vcu_vnav_ah", &self.vcu_vnav_ah())
                .field("vcu_motor_ah", &self.vcu_motor_ah())
                .field("vcu_wsfl_ah", &self.vcu_wsfl_ah())
            .finish()
        } else {
            f.debug_tuple("VcuCoulombCounters").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuCoulombCounters {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_vnav_ah = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_motor_ah = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_wsfl_ah = u.float_in_range(0_f32..=65535_f32)?;
        VcuCoulombCounters::new(vcu_vnav_ah,vcu_motor_ah,vcu_wsfl_ah).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_slip_info
///
/// - ID: 212 (0xd4)
/// - Size: 6 bytes
/// - Transmitter: vcu
///
/// vcu wheel speed and slip info for traction control and launch control
#[derive(Clone, Copy)]
pub struct VcuSlipInfo {
    raw: [u8; 6],
}

impl VcuSlipInfo {
    pub const MESSAGE_ID: u32 = 212;
    
    pub const VCU_CALCULATED_SLIP_MIN: f32 = 0_f32;
    pub const VCU_CALCULATED_SLIP_MAX: f32 = 65535_f32;
    pub const VCU_REAR_RPM_AVG_MIN: f32 = 0_f32;
    pub const VCU_REAR_RPM_AVG_MAX: f32 = 65535_f32;
    pub const VCU_FRONT_RPM_AVG_MIN: f32 = 0_f32;
    pub const VCU_FRONT_RPM_AVG_MAX: f32 = 65535_f32;
    
    /// Construct new vcu_slip_info from values
    pub fn new(vcu_calculated_slip: f32, vcu_rear_rpm_avg: f32, vcu_front_rpm_avg: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 6] };
        res.set_vcu_calculated_slip(vcu_calculated_slip)?;
        res.set_vcu_rear_rpm_avg(vcu_rear_rpm_avg)?;
        res.set_vcu_front_rpm_avg(vcu_front_rpm_avg)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 6] {
        &self.raw
    }
    
    /// vcu_calculated_slip
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_calculated_slip(&self) -> f32 {
        self.vcu_calculated_slip_raw()
    }
    
    /// Get raw value of vcu_calculated_slip
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_calculated_slip_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_calculated_slip
    #[inline(always)]
    pub fn set_vcu_calculated_slip(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 212 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// vcu_rear_rpm_avg
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_rear_rpm_avg(&self) -> f32 {
        self.vcu_rear_rpm_avg_raw()
    }
    
    /// Get raw value of vcu_rear_rpm_avg
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_rear_rpm_avg_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_rear_rpm_avg
    #[inline(always)]
    pub fn set_vcu_rear_rpm_avg(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 212 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// vcu_front_rpm_avg
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_front_rpm_avg(&self) -> f32 {
        self.vcu_front_rpm_avg_raw()
    }
    
    /// Get raw value of vcu_front_rpm_avg
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_front_rpm_avg_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of vcu_front_rpm_avg
    #[inline(always)]
    pub fn set_vcu_front_rpm_avg(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 212 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuSlipInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 6 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 6];
        raw.copy_from_slice(&payload[..6]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuSlipInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuSlipInfo")
                .field("vcu_calculated_slip", &self.vcu_calculated_slip())
                .field("vcu_rear_rpm_avg", &self.vcu_rear_rpm_avg())
                .field("vcu_front_rpm_avg", &self.vcu_front_rpm_avg())
            .finish()
        } else {
            f.debug_tuple("VcuSlipInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuSlipInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_calculated_slip = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_rear_rpm_avg = u.float_in_range(0_f32..=65535_f32)?;
        let vcu_front_rpm_avg = u.float_in_range(0_f32..=65535_f32)?;
        VcuSlipInfo::new(vcu_calculated_slip,vcu_rear_rpm_avg,vcu_front_rpm_avg).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// vcu_set_parameter
///
/// - ID: 214 (0xd6)
/// - Size: 5 bytes
/// - Transmitter: vcu
///
/// The first byte is the target, the last 4 are the value to set
#[derive(Clone, Copy)]
pub struct VcuSetParameter {
    raw: [u8; 5],
}

impl VcuSetParameter {
    pub const MESSAGE_ID: u32 = 214;
    
    pub const VCU_PARAMETER_VALUE_MIN: u32 = 0_u32;
    pub const VCU_PARAMETER_VALUE_MAX: u32 = 4294967295_u32;
    pub const VCU_TARGET_PARAMETER_MIN: u8 = 0_u8;
    pub const VCU_TARGET_PARAMETER_MAX: u8 = 255_u8;
    
    /// Construct new vcu_set_parameter from values
    pub fn new(vcu_parameter_value: u32, vcu_target_parameter: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 5] };
        res.set_vcu_parameter_value(vcu_parameter_value)?;
        res.set_vcu_target_parameter(vcu_target_parameter)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 5] {
        &self.raw
    }
    
    /// vcu_parameter_value
    ///
    /// The actual value that you want to set the parameter to
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_parameter_value(&self) -> u32 {
        self.vcu_parameter_value_raw()
    }
    
    /// Get raw value of vcu_parameter_value
    ///
    /// - Start bit: 8
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_parameter_value_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[8..40].load_le::<u32>();
        
        signal
    }
    
    /// Set value of vcu_parameter_value
    #[inline(always)]
    pub fn set_vcu_parameter_value(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 214 });
        }
        self.raw.view_bits_mut::<Lsb0>()[8..40].store_le(value);
        Ok(())
    }
    
    /// vcu_target_parameter
    ///
    /// look in the vcu.hpp file in the VCU repo for the enum of targets
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_target_parameter(&self) -> u8 {
        self.vcu_target_parameter_raw()
    }
    
    /// Get raw value of vcu_target_parameter
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_target_parameter_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of vcu_target_parameter
    #[inline(always)]
    pub fn set_vcu_target_parameter(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 214 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for VcuSetParameter {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 5 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 5];
        raw.copy_from_slice(&payload[..5]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for VcuSetParameter {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("VcuSetParameter")
                .field("vcu_parameter_value", &self.vcu_parameter_value())
                .field("vcu_target_parameter", &self.vcu_target_parameter())
            .finish()
        } else {
            f.debug_tuple("VcuSetParameter").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for VcuSetParameter {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_parameter_value = u.int_in_range(0..=4294967295)?;
        let vcu_target_parameter = u.int_in_range(0..=255)?;
        VcuSetParameter::new(vcu_parameter_value,vcu_target_parameter).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// dash_buttons
///
/// - ID: 235 (0xeb)
/// - Size: 1 bytes
/// - Transmitter: dash
///
/// status of buttons on the dash
#[derive(Clone, Copy)]
pub struct DashButtons {
    raw: [u8; 1],
}

impl DashButtons {
    pub const MESSAGE_ID: u32 = 235;
    
    
    /// Construct new dash_buttons from values
    pub fn new(dash_button6status: bool, dash_button5status: bool, dash_button4status: bool, dash_button3status: bool, dash_button2status: bool, dash_button1status: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_dash_button6status(dash_button6status)?;
        res.set_dash_button5status(dash_button5status)?;
        res.set_dash_button4status(dash_button4status)?;
        res.set_dash_button3status(dash_button3status)?;
        res.set_dash_button2status(dash_button2status)?;
        res.set_dash_button1status(dash_button1status)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// dash_button6status
    ///
    /// status of button6 on dash (1 = pressed)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn dash_button6status(&self) -> bool {
        self.dash_button6status_raw()
    }
    
    /// Get raw value of dash_button6status
    ///
    /// - Start bit: 5
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dash_button6status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[5..6].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of dash_button6status
    #[inline(always)]
    pub fn set_dash_button6status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[5..6].store_le(value);
        Ok(())
    }
    
    /// dash_button5status
    ///
    /// status of button5 on dash (1 = pressed)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn dash_button5status(&self) -> bool {
        self.dash_button5status_raw()
    }
    
    /// Get raw value of dash_button5status
    ///
    /// - Start bit: 4
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dash_button5status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[4..5].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of dash_button5status
    #[inline(always)]
    pub fn set_dash_button5status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[4..5].store_le(value);
        Ok(())
    }
    
    /// dash_button4status
    ///
    /// status of button4 on dash (1 = pressed)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn dash_button4status(&self) -> bool {
        self.dash_button4status_raw()
    }
    
    /// Get raw value of dash_button4status
    ///
    /// - Start bit: 3
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dash_button4status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[3..4].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of dash_button4status
    #[inline(always)]
    pub fn set_dash_button4status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[3..4].store_le(value);
        Ok(())
    }
    
    /// dash_button3status
    ///
    /// status of button3 on dash (1 = pressed)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn dash_button3status(&self) -> bool {
        self.dash_button3status_raw()
    }
    
    /// Get raw value of dash_button3status
    ///
    /// - Start bit: 2
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dash_button3status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[2..3].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of dash_button3status
    #[inline(always)]
    pub fn set_dash_button3status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[2..3].store_le(value);
        Ok(())
    }
    
    /// dash_button2status
    ///
    /// status of button2 on dash (1 = pressed)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn dash_button2status(&self) -> bool {
        self.dash_button2status_raw()
    }
    
    /// Get raw value of dash_button2status
    ///
    /// - Start bit: 1
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dash_button2status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[1..2].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of dash_button2status
    #[inline(always)]
    pub fn set_dash_button2status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[1..2].store_le(value);
        Ok(())
    }
    
    /// dash_button1status
    ///
    /// status of button1 on dash (1 = pressed)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn dash_button1status(&self) -> bool {
        self.dash_button1status_raw()
    }
    
    /// Get raw value of dash_button1status
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dash_button1status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of dash_button1status
    #[inline(always)]
    pub fn set_dash_button1status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for DashButtons {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for DashButtons {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("DashButtons")
                .field("dash_button6status", &self.dash_button6status())
                .field("dash_button5status", &self.dash_button5status())
                .field("dash_button4status", &self.dash_button4status())
                .field("dash_button3status", &self.dash_button3status())
                .field("dash_button2status", &self.dash_button2status())
                .field("dash_button1status", &self.dash_button1status())
            .finish()
        } else {
            f.debug_tuple("DashButtons").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for DashButtons {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let dash_button6status = u.int_in_range(0..=1)? == 1;
        let dash_button5status = u.int_in_range(0..=1)? == 1;
        let dash_button4status = u.int_in_range(0..=1)? == 1;
        let dash_button3status = u.int_in_range(0..=1)? == 1;
        let dash_button2status = u.int_in_range(0..=1)? == 1;
        let dash_button1status = u.int_in_range(0..=1)? == 1;
        DashButtons::new(dash_button6status,dash_button5status,dash_button4status,dash_button3status,dash_button2status,dash_button1status).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// dash_board_data
///
/// - ID: 236 (0xec)
/// - Size: 8 bytes
/// - Transmitter: dash
///
/// information on the running dash firmware and system on-timer
#[derive(Clone, Copy)]
pub struct DashBoardData {
    raw: [u8; 8],
}

impl DashBoardData {
    pub const MESSAGE_ID: u32 = 236;
    
    pub const BOARD_ON_TIME_SECONDS_MIN: u16 = 0_u16;
    pub const BOARD_ON_TIME_SECONDS_MAX: u16 = 65535_u16;
    pub const FIRMWARE_VERSION_MIN: u32 = 0_u32;
    pub const FIRMWARE_VERSION_MAX: u32 = 4294967295_u32;
    
    /// Construct new dash_board_data from values
    pub fn new(firmware_is_dirty: bool, firmware_on_main: bool, board_on_time_seconds: u16, firmware_version: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_firmware_is_dirty(firmware_is_dirty)?;
        res.set_firmware_on_main(firmware_on_main)?;
        res.set_board_on_time_seconds(board_on_time_seconds)?;
        res.set_firmware_version(firmware_version)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// firmware_is_dirty
    ///
    /// if the compiled firmware had uncommitted changes (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_is_dirty(&self) -> bool {
        self.firmware_is_dirty_raw()
    }
    
    /// Get raw value of firmware_is_dirty
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_is_dirty_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_is_dirty
    #[inline(always)]
    pub fn set_firmware_is_dirty(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// firmware_on_main
    ///
    /// if the compiled firmware was from main branch (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_on_main(&self) -> bool {
        self.firmware_on_main_raw()
    }
    
    /// Get raw value of firmware_on_main
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_on_main_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_on_main
    #[inline(always)]
    pub fn set_firmware_on_main(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// board_on_time_seconds
    ///
    /// time in seconds which the board has been powered on and running
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_on_time_seconds(&self) -> u16 {
        self.board_on_time_seconds_raw()
    }
    
    /// Get raw value of board_on_time_seconds
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_on_time_seconds_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of board_on_time_seconds
    #[inline(always)]
    pub fn set_board_on_time_seconds(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 236 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// firmware_version
    ///
    /// short hash of the firmware
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version_raw()
    }
    
    /// Get raw value of firmware_version
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_version_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of firmware_version
    #[inline(always)]
    pub fn set_firmware_version(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 236 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for DashBoardData {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for DashBoardData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("DashBoardData")
                .field("firmware_is_dirty", &self.firmware_is_dirty())
                .field("firmware_on_main", &self.firmware_on_main())
                .field("board_on_time_seconds", &self.board_on_time_seconds())
                .field("firmware_version", &self.firmware_version())
            .finish()
        } else {
            f.debug_tuple("DashBoardData").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for DashBoardData {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let firmware_is_dirty = u.int_in_range(0..=1)? == 1;
        let firmware_on_main = u.int_in_range(0..=1)? == 1;
        let board_on_time_seconds = u.int_in_range(0..=65535)?;
        let firmware_version = u.int_in_range(0..=4294967295)?;
        DashBoardData::new(firmware_is_dirty,firmware_on_main,board_on_time_seconds,firmware_version).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// evelogger_vectornav_attitude
///
/// - ID: 500 (0x1f4)
/// - Size: 6 bytes
/// - Transmitter: evelogger
///
/// vectornav attitude readings
#[derive(Clone, Copy)]
pub struct EveloggerVectornavAttitude {
    raw: [u8; 6],
}

impl EveloggerVectornavAttitude {
    pub const MESSAGE_ID: u32 = 500;
    
    pub const EVELOGGER_VECTORNAV_PITCH_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_PITCH_MAX: f32 = 65535_f32;
    pub const EVELOGGER_VECTORNAV_ROLL_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_ROLL_MAX: f32 = 65535_f32;
    pub const EVELOGGER_VECTORNAV_YAW_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_YAW_MAX: f32 = 65535_f32;
    
    /// Construct new evelogger_vectornav_attitude from values
    pub fn new(evelogger_vectornav_pitch: f32, evelogger_vectornav_roll: f32, evelogger_vectornav_yaw: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 6] };
        res.set_evelogger_vectornav_pitch(evelogger_vectornav_pitch)?;
        res.set_evelogger_vectornav_roll(evelogger_vectornav_roll)?;
        res.set_evelogger_vectornav_yaw(evelogger_vectornav_yaw)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 6] {
        &self.raw
    }
    
    /// evelogger_vectornav_pitch
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_pitch(&self) -> f32 {
        self.evelogger_vectornav_pitch_raw()
    }
    
    /// Get raw value of evelogger_vectornav_pitch
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_pitch_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_pitch
    #[inline(always)]
    pub fn set_evelogger_vectornav_pitch(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 500 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_roll
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_roll(&self) -> f32 {
        self.evelogger_vectornav_roll_raw()
    }
    
    /// Get raw value of evelogger_vectornav_roll
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_roll_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_roll
    #[inline(always)]
    pub fn set_evelogger_vectornav_roll(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 500 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_yaw
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_yaw(&self) -> f32 {
        self.evelogger_vectornav_yaw_raw()
    }
    
    /// Get raw value of evelogger_vectornav_yaw
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_yaw_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_yaw
    #[inline(always)]
    pub fn set_evelogger_vectornav_yaw(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 500 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for EveloggerVectornavAttitude {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 6 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 6];
        raw.copy_from_slice(&payload[..6]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for EveloggerVectornavAttitude {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("EveloggerVectornavAttitude")
                .field("evelogger_vectornav_pitch", &self.evelogger_vectornav_pitch())
                .field("evelogger_vectornav_roll", &self.evelogger_vectornav_roll())
                .field("evelogger_vectornav_yaw", &self.evelogger_vectornav_yaw())
            .finish()
        } else {
            f.debug_tuple("EveloggerVectornavAttitude").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for EveloggerVectornavAttitude {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let evelogger_vectornav_pitch = u.float_in_range(0_f32..=65535_f32)?;
        let evelogger_vectornav_roll = u.float_in_range(0_f32..=65535_f32)?;
        let evelogger_vectornav_yaw = u.float_in_range(0_f32..=65535_f32)?;
        EveloggerVectornavAttitude::new(evelogger_vectornav_pitch,evelogger_vectornav_roll,evelogger_vectornav_yaw).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// evelogger_vectornav_gyro
///
/// - ID: 501 (0x1f5)
/// - Size: 6 bytes
/// - Transmitter: evelogger
///
/// vectornav gyro readings
#[derive(Clone, Copy)]
pub struct EveloggerVectornavGyro {
    raw: [u8; 6],
}

impl EveloggerVectornavGyro {
    pub const MESSAGE_ID: u32 = 501;
    
    pub const EVELOGGER_VECTORNAV_W_Z_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_W_Z_MAX: f32 = 65535_f32;
    pub const EVELOGGER_VECTORNAV_W_Y_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_W_Y_MAX: f32 = 65535_f32;
    pub const EVELOGGER_VECTORNAV_W_X_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_W_X_MAX: f32 = 65535_f32;
    
    /// Construct new evelogger_vectornav_gyro from values
    pub fn new(evelogger_vectornav_w_z: f32, evelogger_vectornav_w_y: f32, evelogger_vectornav_w_x: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 6] };
        res.set_evelogger_vectornav_w_z(evelogger_vectornav_w_z)?;
        res.set_evelogger_vectornav_w_y(evelogger_vectornav_w_y)?;
        res.set_evelogger_vectornav_w_x(evelogger_vectornav_w_x)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 6] {
        &self.raw
    }
    
    /// evelogger_vectornav_W_z
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_w_z(&self) -> f32 {
        self.evelogger_vectornav_w_z_raw()
    }
    
    /// Get raw value of evelogger_vectornav_W_z
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_w_z_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_W_z
    #[inline(always)]
    pub fn set_evelogger_vectornav_w_z(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 501 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_W_y
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_w_y(&self) -> f32 {
        self.evelogger_vectornav_w_y_raw()
    }
    
    /// Get raw value of evelogger_vectornav_W_y
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_w_y_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_W_y
    #[inline(always)]
    pub fn set_evelogger_vectornav_w_y(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 501 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_W_x
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_w_x(&self) -> f32 {
        self.evelogger_vectornav_w_x_raw()
    }
    
    /// Get raw value of evelogger_vectornav_W_x
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_w_x_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_W_x
    #[inline(always)]
    pub fn set_evelogger_vectornav_w_x(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 501 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for EveloggerVectornavGyro {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 6 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 6];
        raw.copy_from_slice(&payload[..6]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for EveloggerVectornavGyro {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("EveloggerVectornavGyro")
                .field("evelogger_vectornav_w_z", &self.evelogger_vectornav_w_z())
                .field("evelogger_vectornav_w_y", &self.evelogger_vectornav_w_y())
                .field("evelogger_vectornav_w_x", &self.evelogger_vectornav_w_x())
            .finish()
        } else {
            f.debug_tuple("EveloggerVectornavGyro").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for EveloggerVectornavGyro {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let evelogger_vectornav_w_z = u.float_in_range(0_f32..=65535_f32)?;
        let evelogger_vectornav_w_y = u.float_in_range(0_f32..=65535_f32)?;
        let evelogger_vectornav_w_x = u.float_in_range(0_f32..=65535_f32)?;
        EveloggerVectornavGyro::new(evelogger_vectornav_w_z,evelogger_vectornav_w_y,evelogger_vectornav_w_x).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// evelogger_vectornav_position
///
/// - ID: 502 (0x1f6)
/// - Size: 8 bytes
/// - Transmitter: evelogger
///
/// vectornav position
#[derive(Clone, Copy)]
pub struct EveloggerVectornavPosition {
    raw: [u8; 8],
}

impl EveloggerVectornavPosition {
    pub const MESSAGE_ID: u32 = 502;
    
    pub const EVELOGGER_VECTORNAV_LONGITUDE_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_LONGITUDE_MAX: f32 = 4294967295_f32;
    pub const EVELOGGER_VECTORNAV_LATITUDE_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_LATITUDE_MAX: f32 = 4294967295_f32;
    
    /// Construct new evelogger_vectornav_position from values
    pub fn new(evelogger_vectornav_longitude: f32, evelogger_vectornav_latitude: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_evelogger_vectornav_longitude(evelogger_vectornav_longitude)?;
        res.set_evelogger_vectornav_latitude(evelogger_vectornav_latitude)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// evelogger_vectornav_longitude
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_longitude(&self) -> f32 {
        self.evelogger_vectornav_longitude_raw()
    }
    
    /// Get raw value of evelogger_vectornav_longitude
    ///
    /// - Start bit: 32
    /// - Signal size: 32 bits
    /// - Factor: 0.0000001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_longitude_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..64].load_le::<u32>();
        
        let signal  = i32::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.0000001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_longitude
    #[inline(always)]
    pub fn set_evelogger_vectornav_longitude(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 4294967295_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 502 });
        }
        let factor = 0.0000001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i32;
        
        let value = u32::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..64].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_latitude
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_latitude(&self) -> f32 {
        self.evelogger_vectornav_latitude_raw()
    }
    
    /// Get raw value of evelogger_vectornav_latitude
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 0.0000001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_latitude_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        let signal  = i32::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.0000001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_latitude
    #[inline(always)]
    pub fn set_evelogger_vectornav_latitude(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 4294967295_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 502 });
        }
        let factor = 0.0000001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i32;
        
        let value = u32::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for EveloggerVectornavPosition {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for EveloggerVectornavPosition {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("EveloggerVectornavPosition")
                .field("evelogger_vectornav_longitude", &self.evelogger_vectornav_longitude())
                .field("evelogger_vectornav_latitude", &self.evelogger_vectornav_latitude())
            .finish()
        } else {
            f.debug_tuple("EveloggerVectornavPosition").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for EveloggerVectornavPosition {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let evelogger_vectornav_longitude = u.float_in_range(0_f32..=4294967295_f32)?;
        let evelogger_vectornav_latitude = u.float_in_range(0_f32..=4294967295_f32)?;
        EveloggerVectornavPosition::new(evelogger_vectornav_longitude,evelogger_vectornav_latitude).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// evelogger_vectornav_velocity
///
/// - ID: 503 (0x1f7)
/// - Size: 6 bytes
/// - Transmitter: evelogger
///
/// vectornav velocity readings
#[derive(Clone, Copy)]
pub struct EveloggerVectornavVelocity {
    raw: [u8; 6],
}

impl EveloggerVectornavVelocity {
    pub const MESSAGE_ID: u32 = 503;
    
    pub const EVELOGGER_VECTORNAV_V_D_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_V_D_MAX: f32 = 65535_f32;
    pub const EVELOGGER_VECTORNAV_V_E_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_V_E_MAX: f32 = 65535_f32;
    pub const EVELOGGER_VECTORNAV_V_N_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_V_N_MAX: f32 = 65535_f32;
    
    /// Construct new evelogger_vectornav_velocity from values
    pub fn new(evelogger_vectornav_v_d: f32, evelogger_vectornav_v_e: f32, evelogger_vectornav_v_n: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 6] };
        res.set_evelogger_vectornav_v_d(evelogger_vectornav_v_d)?;
        res.set_evelogger_vectornav_v_e(evelogger_vectornav_v_e)?;
        res.set_evelogger_vectornav_v_n(evelogger_vectornav_v_n)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 6] {
        &self.raw
    }
    
    /// evelogger_vectornav_v_d
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_v_d(&self) -> f32 {
        self.evelogger_vectornav_v_d_raw()
    }
    
    /// Get raw value of evelogger_vectornav_v_d
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_v_d_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_v_d
    #[inline(always)]
    pub fn set_evelogger_vectornav_v_d(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 503 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_v_e
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_v_e(&self) -> f32 {
        self.evelogger_vectornav_v_e_raw()
    }
    
    /// Get raw value of evelogger_vectornav_v_e
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_v_e_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_v_e
    #[inline(always)]
    pub fn set_evelogger_vectornav_v_e(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 503 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_v_n
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_v_n(&self) -> f32 {
        self.evelogger_vectornav_v_n_raw()
    }
    
    /// Get raw value of evelogger_vectornav_v_n
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_v_n_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_v_n
    #[inline(always)]
    pub fn set_evelogger_vectornav_v_n(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 503 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for EveloggerVectornavVelocity {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 6 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 6];
        raw.copy_from_slice(&payload[..6]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for EveloggerVectornavVelocity {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("EveloggerVectornavVelocity")
                .field("evelogger_vectornav_v_d", &self.evelogger_vectornav_v_d())
                .field("evelogger_vectornav_v_e", &self.evelogger_vectornav_v_e())
                .field("evelogger_vectornav_v_n", &self.evelogger_vectornav_v_n())
            .finish()
        } else {
            f.debug_tuple("EveloggerVectornavVelocity").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for EveloggerVectornavVelocity {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let evelogger_vectornav_v_d = u.float_in_range(0_f32..=65535_f32)?;
        let evelogger_vectornav_v_e = u.float_in_range(0_f32..=65535_f32)?;
        let evelogger_vectornav_v_n = u.float_in_range(0_f32..=65535_f32)?;
        EveloggerVectornavVelocity::new(evelogger_vectornav_v_d,evelogger_vectornav_v_e,evelogger_vectornav_v_n).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// evelogger_vectornav_acceleration
///
/// - ID: 504 (0x1f8)
/// - Size: 6 bytes
/// - Transmitter: evelogger
///
/// vectornav accelerometer readings
#[derive(Clone, Copy)]
pub struct EveloggerVectornavAcceleration {
    raw: [u8; 6],
}

impl EveloggerVectornavAcceleration {
    pub const MESSAGE_ID: u32 = 504;
    
    pub const EVELOGGER_VECTORNAV_ACCEL_Z_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_ACCEL_Z_MAX: f32 = 65535_f32;
    pub const EVELOGGER_VECTORNAV_ACCEL_Y_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_ACCEL_Y_MAX: f32 = 65535_f32;
    pub const EVELOGGER_VECTORNAV_ACCEL_X_MIN: f32 = 0_f32;
    pub const EVELOGGER_VECTORNAV_ACCEL_X_MAX: f32 = 65535_f32;
    
    /// Construct new evelogger_vectornav_acceleration from values
    pub fn new(evelogger_vectornav_accel_z: f32, evelogger_vectornav_accel_y: f32, evelogger_vectornav_accel_x: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 6] };
        res.set_evelogger_vectornav_accel_z(evelogger_vectornav_accel_z)?;
        res.set_evelogger_vectornav_accel_y(evelogger_vectornav_accel_y)?;
        res.set_evelogger_vectornav_accel_x(evelogger_vectornav_accel_x)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 6] {
        &self.raw
    }
    
    /// evelogger_vectornav_accelZ
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_accel_z(&self) -> f32 {
        self.evelogger_vectornav_accel_z_raw()
    }
    
    /// Get raw value of evelogger_vectornav_accelZ
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_accel_z_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_accelZ
    #[inline(always)]
    pub fn set_evelogger_vectornav_accel_z(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 504 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_accelY
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_accel_y(&self) -> f32 {
        self.evelogger_vectornav_accel_y_raw()
    }
    
    /// Get raw value of evelogger_vectornav_accelY
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_accel_y_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_accelY
    #[inline(always)]
    pub fn set_evelogger_vectornav_accel_y(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 504 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// evelogger_vectornav_accelX
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn evelogger_vectornav_accel_x(&self) -> f32 {
        self.evelogger_vectornav_accel_x_raw()
    }
    
    /// Get raw value of evelogger_vectornav_accelX
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn evelogger_vectornav_accel_x_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of evelogger_vectornav_accelX
    #[inline(always)]
    pub fn set_evelogger_vectornav_accel_x(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 504 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for EveloggerVectornavAcceleration {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 6 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 6];
        raw.copy_from_slice(&payload[..6]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for EveloggerVectornavAcceleration {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("EveloggerVectornavAcceleration")
                .field("evelogger_vectornav_accel_z", &self.evelogger_vectornav_accel_z())
                .field("evelogger_vectornav_accel_y", &self.evelogger_vectornav_accel_y())
                .field("evelogger_vectornav_accel_x", &self.evelogger_vectornav_accel_x())
            .finish()
        } else {
            f.debug_tuple("EveloggerVectornavAcceleration").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for EveloggerVectornavAcceleration {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let evelogger_vectornav_accel_z = u.float_in_range(0_f32..=65535_f32)?;
        let evelogger_vectornav_accel_y = u.float_in_range(0_f32..=65535_f32)?;
        let evelogger_vectornav_accel_x = u.float_in_range(0_f32..=65535_f32)?;
        EveloggerVectornavAcceleration::new(evelogger_vectornav_accel_z,evelogger_vectornav_accel_y,evelogger_vectornav_accel_x).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// evelogger_vectornav_time
///
/// - ID: 505 (0x1f9)
/// - Size: 8 bytes
/// - Transmitter: evelogger
///
/// vectornav breakout realtime
#[derive(Clone, Copy)]
pub struct EveloggerVectornavTime {
    raw: [u8; 8],
}

impl EveloggerVectornavTime {
    pub const MESSAGE_ID: u32 = 505;
    
    pub const UNIX_TIME_NS_MIN: u64 = 0_u64;
    pub const UNIX_TIME_NS_MAX: u64 = 18446744073709552000_u64;
    
    /// Construct new evelogger_vectornav_time from values
    pub fn new(unix_time_ns: u64) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_unix_time_ns(unix_time_ns)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// unix_time_ns
    ///
    /// Time in nanoseconds since January 1st 1970
    ///
    /// - Min: 0
    /// - Max: 18446744073709552000
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn unix_time_ns(&self) -> u64 {
        self.unix_time_ns_raw()
    }
    
    /// Get raw value of unix_time_ns
    ///
    /// - Start bit: 0
    /// - Signal size: 64 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn unix_time_ns_raw(&self) -> u64 {
        let signal = self.raw.view_bits::<Lsb0>()[0..64].load_le::<u64>();
        
        signal
    }
    
    /// Set value of unix_time_ns
    #[inline(always)]
    pub fn set_unix_time_ns(&mut self, value: u64) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u64 || 18446744073709552000_u64 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 505 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..64].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for EveloggerVectornavTime {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for EveloggerVectornavTime {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("EveloggerVectornavTime")
                .field("unix_time_ns", &self.unix_time_ns())
            .finish()
        } else {
            f.debug_tuple("EveloggerVectornavTime").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for EveloggerVectornavTime {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let unix_time_ns = u.int_in_range(0..=18446744073709552000)?;
        EveloggerVectornavTime::new(unix_time_ns).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// acu_shutdown_status
///
/// - ID: 600 (0x258)
/// - Size: 6 bytes
/// - Transmitter: acu
///
/// information on IMD and BMS shutdown circuit relays, as well as IMD diagnostic info
#[derive(Clone, Copy)]
pub struct AcuShutdownStatus {
    raw: [u8; 6],
}

impl AcuShutdownStatus {
    pub const MESSAGE_ID: u32 = 600;
    
    pub const ACU_IMD_PWM_DUTY_MIN: u8 = 0_u8;
    pub const ACU_IMD_PWM_DUTY_MAX: u8 = 255_u8;
    pub const ACU_IMD_PWM_FREQUENCY_MIN: u8 = 0_u8;
    pub const ACU_IMD_PWM_FREQUENCY_MAX: u8 = 255_u8;
    
    /// Construct new acu_shutdown_status from values
    pub fn new(acu_imd_pwm_duty: u8, acu_imd_pwm_frequency: u8, acu_bms_gpio_state: bool, acu_imd_gpio_state: bool, acu_bms_relay_state: bool, acu_imd_relay_state: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 6] };
        res.set_acu_imd_pwm_duty(acu_imd_pwm_duty)?;
        res.set_acu_imd_pwm_frequency(acu_imd_pwm_frequency)?;
        res.set_acu_bms_gpio_state(acu_bms_gpio_state)?;
        res.set_acu_imd_gpio_state(acu_imd_gpio_state)?;
        res.set_acu_bms_relay_state(acu_bms_relay_state)?;
        res.set_acu_imd_relay_state(acu_imd_relay_state)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 6] {
        &self.raw
    }
    
    /// acu_imd_pwm_duty
    ///
    /// duty cycle of IMD: ~5% on normal conditions
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_imd_pwm_duty(&self) -> u8 {
        self.acu_imd_pwm_duty_raw()
    }
    
    /// Get raw value of acu_imd_pwm_duty
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_imd_pwm_duty_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        signal
    }
    
    /// Set value of acu_imd_pwm_duty
    #[inline(always)]
    pub fn set_acu_imd_pwm_duty(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 600 });
        }
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// acu_imd_pwm_frequency
    ///
    /// PWM frequency of the IMD: 10hz on normal operation
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_imd_pwm_frequency(&self) -> u8 {
        self.acu_imd_pwm_frequency_raw()
    }
    
    /// Get raw value of acu_imd_pwm_frequency
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_imd_pwm_frequency_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        signal
    }
    
    /// Set value of acu_imd_pwm_frequency
    #[inline(always)]
    pub fn set_acu_imd_pwm_frequency(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 600 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// acu_bms_gpio_state
    ///
    /// Status of the BMS ENABLE output: 1 is OK, 0 is FAULTED
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_bms_gpio_state(&self) -> bool {
        self.acu_bms_gpio_state_raw()
    }
    
    /// Get raw value of acu_bms_gpio_state
    ///
    /// - Start bit: 31
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_bms_gpio_state_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[31..32].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of acu_bms_gpio_state
    #[inline(always)]
    pub fn set_acu_bms_gpio_state(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[31..32].store_le(value);
        Ok(())
    }
    
    /// acu_imd_gpio_state
    ///
    /// Status of the IMD OKHS output: 1 is OK, 0 is FAULTED
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_imd_gpio_state(&self) -> bool {
        self.acu_imd_gpio_state_raw()
    }
    
    /// Get raw value of acu_imd_gpio_state
    ///
    /// - Start bit: 23
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_imd_gpio_state_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[23..24].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of acu_imd_gpio_state
    #[inline(always)]
    pub fn set_acu_imd_gpio_state(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[23..24].store_le(value);
        Ok(())
    }
    
    /// acu_bms_relay_state
    ///
    /// Status of the BMS relay as sensed by the ACU: 1 is closed, 0 is open
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_bms_relay_state(&self) -> bool {
        self.acu_bms_relay_state_raw()
    }
    
    /// Get raw value of acu_bms_relay_state
    ///
    /// - Start bit: 15
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_bms_relay_state_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[15..16].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of acu_bms_relay_state
    #[inline(always)]
    pub fn set_acu_bms_relay_state(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[15..16].store_le(value);
        Ok(())
    }
    
    /// acu_imd_relay_state
    ///
    /// Status of the IMD relay as sensed by the ACU: 1 is closed, 0 is open
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_imd_relay_state(&self) -> bool {
        self.acu_imd_relay_state_raw()
    }
    
    /// Get raw value of acu_imd_relay_state
    ///
    /// - Start bit: 7
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_imd_relay_state_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[7..8].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of acu_imd_relay_state
    #[inline(always)]
    pub fn set_acu_imd_relay_state(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[7..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for AcuShutdownStatus {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 6 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 6];
        raw.copy_from_slice(&payload[..6]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for AcuShutdownStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("AcuShutdownStatus")
                .field("acu_imd_pwm_duty", &self.acu_imd_pwm_duty())
                .field("acu_imd_pwm_frequency", &self.acu_imd_pwm_frequency())
                .field("acu_bms_gpio_state", &self.acu_bms_gpio_state())
                .field("acu_imd_gpio_state", &self.acu_imd_gpio_state())
                .field("acu_bms_relay_state", &self.acu_bms_relay_state())
                .field("acu_imd_relay_state", &self.acu_imd_relay_state())
            .finish()
        } else {
            f.debug_tuple("AcuShutdownStatus").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for AcuShutdownStatus {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let acu_imd_pwm_duty = u.int_in_range(0..=255)?;
        let acu_imd_pwm_frequency = u.int_in_range(0..=255)?;
        let acu_bms_gpio_state = u.int_in_range(0..=1)? == 1;
        let acu_imd_gpio_state = u.int_in_range(0..=1)? == 1;
        let acu_bms_relay_state = u.int_in_range(0..=1)? == 1;
        let acu_imd_relay_state = u.int_in_range(0..=1)? == 1;
        AcuShutdownStatus::new(acu_imd_pwm_duty,acu_imd_pwm_frequency,acu_bms_gpio_state,acu_imd_gpio_state,acu_bms_relay_state,acu_imd_relay_state).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// acu_board_voltage_readings
///
/// - ID: 601 (0x259)
/// - Size: 6 bytes
/// - Transmitter: acu
///
/// ACU voltage and current measurements
#[derive(Clone, Copy)]
pub struct AcuBoardVoltageReadings {
    raw: [u8; 6],
}

impl AcuBoardVoltageReadings {
    pub const MESSAGE_ID: u32 = 601;
    
    pub const ACU_3V_VOLTAGE_MIN: f32 = 0_f32;
    pub const ACU_3V_VOLTAGE_MAX: f32 = 255_f32;
    pub const ACU_5V_VOLTAGE_MIN: f32 = 0_f32;
    pub const ACU_5V_VOLTAGE_MAX: f32 = 255_f32;
    pub const ACU_GLV_CURRENT_MIN: f32 = 0_f32;
    pub const ACU_GLV_CURRENT_MAX: f32 = 255_f32;
    pub const ACU_GLV_VOLTAGE_MIN: f32 = 0_f32;
    pub const ACU_GLV_VOLTAGE_MAX: f32 = 255_f32;
    pub const ACU_SDC_CURRENT_MIN: f32 = 0_f32;
    pub const ACU_SDC_CURRENT_MAX: f32 = 255_f32;
    pub const ACU_SDC_VOLTAGE_MIN: f32 = 0_f32;
    pub const ACU_SDC_VOLTAGE_MAX: f32 = 255_f32;
    
    /// Construct new acu_board_voltage_readings from values
    pub fn new(acu_3v_voltage: f32, acu_5v_voltage: f32, acu_glv_current: f32, acu_glv_voltage: f32, acu_sdc_current: f32, acu_sdc_voltage: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 6] };
        res.set_acu_3v_voltage(acu_3v_voltage)?;
        res.set_acu_5v_voltage(acu_5v_voltage)?;
        res.set_acu_glv_current(acu_glv_current)?;
        res.set_acu_glv_voltage(acu_glv_voltage)?;
        res.set_acu_sdc_current(acu_sdc_current)?;
        res.set_acu_sdc_voltage(acu_sdc_voltage)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 6] {
        &self.raw
    }
    
    /// acu_3v_voltage
    ///
    /// ACU 3v rail reading
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_3v_voltage(&self) -> f32 {
        self.acu_3v_voltage_raw()
    }
    
    /// Get raw value of acu_3v_voltage
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 0.01294
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_3v_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        let factor = 0.01294_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of acu_3v_voltage
    #[inline(always)]
    pub fn set_acu_3v_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 601 });
        }
        let factor = 0.01294_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// acu_5v_voltage
    ///
    /// ACU 5v rail reading
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_5v_voltage(&self) -> f32 {
        self.acu_5v_voltage_raw()
    }
    
    /// Get raw value of acu_5v_voltage
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 0.01294
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_5v_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        let factor = 0.01294_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of acu_5v_voltage
    #[inline(always)]
    pub fn set_acu_5v_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 601 });
        }
        let factor = 0.01294_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// acu_glv_current
    ///
    /// ACU GLV current reading from the ACU
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: "amps"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_glv_current(&self) -> f32 {
        self.acu_glv_current_raw()
    }
    
    /// Get raw value of acu_glv_current
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 0.03235
    /// - Offset: -6.25
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_glv_current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        let factor = 0.03235_f32;
        let offset = -6.25_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of acu_glv_current
    #[inline(always)]
    pub fn set_acu_glv_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 601 });
        }
        let factor = 0.03235_f32;
        let offset = -6.25_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// acu_glv_voltage
    ///
    /// ACU GLV system voltage reading
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_glv_voltage(&self) -> f32 {
        self.acu_glv_voltage_raw()
    }
    
    /// Get raw value of acu_glv_voltage
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 0.01294
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_glv_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        let factor = 0.01294_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of acu_glv_voltage
    #[inline(always)]
    pub fn set_acu_glv_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 601 });
        }
        let factor = 0.01294_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// acu_sdc_current
    ///
    /// ACU shutdown circuit current reading thru the ACU
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: "amps"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_sdc_current(&self) -> f32 {
        self.acu_sdc_current_raw()
    }
    
    /// Get raw value of acu_sdc_current
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 0.03235
    /// - Offset: -6.25
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_sdc_current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        let factor = 0.03235_f32;
        let offset = -6.25_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of acu_sdc_current
    #[inline(always)]
    pub fn set_acu_sdc_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 601 });
        }
        let factor = 0.03235_f32;
        let offset = -6.25_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// acu_sdc_voltage
    ///
    /// ACU GLV system voltage reading
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn acu_sdc_voltage(&self) -> f32 {
        self.acu_sdc_voltage_raw()
    }
    
    /// Get raw value of acu_sdc_voltage
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 0.01294
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn acu_sdc_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        let factor = 0.01294_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of acu_sdc_voltage
    #[inline(always)]
    pub fn set_acu_sdc_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 255_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 601 });
        }
        let factor = 0.01294_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for AcuBoardVoltageReadings {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 6 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 6];
        raw.copy_from_slice(&payload[..6]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for AcuBoardVoltageReadings {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("AcuBoardVoltageReadings")
                .field("acu_3v_voltage", &self.acu_3v_voltage())
                .field("acu_5v_voltage", &self.acu_5v_voltage())
                .field("acu_glv_current", &self.acu_glv_current())
                .field("acu_glv_voltage", &self.acu_glv_voltage())
                .field("acu_sdc_current", &self.acu_sdc_current())
                .field("acu_sdc_voltage", &self.acu_sdc_voltage())
            .finish()
        } else {
            f.debug_tuple("AcuBoardVoltageReadings").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for AcuBoardVoltageReadings {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let acu_3v_voltage = u.float_in_range(0_f32..=255_f32)?;
        let acu_5v_voltage = u.float_in_range(0_f32..=255_f32)?;
        let acu_glv_current = u.float_in_range(0_f32..=255_f32)?;
        let acu_glv_voltage = u.float_in_range(0_f32..=255_f32)?;
        let acu_sdc_current = u.float_in_range(0_f32..=255_f32)?;
        let acu_sdc_voltage = u.float_in_range(0_f32..=255_f32)?;
        AcuBoardVoltageReadings::new(acu_3v_voltage,acu_5v_voltage,acu_glv_current,acu_glv_voltage,acu_sdc_current,acu_sdc_voltage).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// acu_board_data
///
/// - ID: 602 (0x25a)
/// - Size: 8 bytes
/// - Transmitter: acu
///
/// information on the running acu firmware and system on-timer
#[derive(Clone, Copy)]
pub struct AcuBoardData {
    raw: [u8; 8],
}

impl AcuBoardData {
    pub const MESSAGE_ID: u32 = 602;
    
    pub const BOARD_HUMIDITY_MIN: u8 = 0_u8;
    pub const BOARD_HUMIDITY_MAX: u8 = 127_u8;
    pub const BOARD_TEMPERATURE_MIN: u8 = 0_u8;
    pub const BOARD_TEMPERATURE_MAX: u8 = 127_u8;
    pub const BOARD_ON_TIME_SECONDS_MIN: u16 = 0_u16;
    pub const BOARD_ON_TIME_SECONDS_MAX: u16 = 65535_u16;
    pub const FIRMWARE_VERSION_MIN: u32 = 0_u32;
    pub const FIRMWARE_VERSION_MAX: u32 = 4294967295_u32;
    
    /// Construct new acu_board_data from values
    pub fn new(board_humidity: u8, board_temperature: u8, firmware_is_dirty: bool, firmware_on_main: bool, board_on_time_seconds: u16, firmware_version: u32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_board_humidity(board_humidity)?;
        res.set_board_temperature(board_temperature)?;
        res.set_firmware_is_dirty(firmware_is_dirty)?;
        res.set_firmware_on_main(firmware_on_main)?;
        res.set_board_on_time_seconds(board_on_time_seconds)?;
        res.set_firmware_version(firmware_version)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// board_humidity
    ///
    /// the relative humidity at the acu
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "relative_humidity"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_humidity(&self) -> u8 {
        self.board_humidity_raw()
    }
    
    /// Get raw value of board_humidity
    ///
    /// - Start bit: 57
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_humidity_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[57..64].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_humidity
    #[inline(always)]
    pub fn set_board_humidity(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 602 });
        }
        self.raw.view_bits_mut::<Lsb0>()[57..64].store_le(value);
        Ok(())
    }
    
    /// board_temperature
    ///
    /// The ambient temperature at the PCB
    ///
    /// - Min: 0
    /// - Max: 127
    /// - Unit: "celcius"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_temperature(&self) -> u8 {
        self.board_temperature_raw()
    }
    
    /// Get raw value of board_temperature
    ///
    /// - Start bit: 50
    /// - Signal size: 7 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[50..57].load_le::<u8>();
        
        signal
    }
    
    /// Set value of board_temperature
    #[inline(always)]
    pub fn set_board_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 127_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 602 });
        }
        self.raw.view_bits_mut::<Lsb0>()[50..57].store_le(value);
        Ok(())
    }
    
    /// firmware_is_dirty
    ///
    /// if the compiled firmware had uncommitted changes (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_is_dirty(&self) -> bool {
        self.firmware_is_dirty_raw()
    }
    
    /// Get raw value of firmware_is_dirty
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_is_dirty_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_is_dirty
    #[inline(always)]
    pub fn set_firmware_is_dirty(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// firmware_on_main
    ///
    /// if the compiled firmware was from main branch (1) or not (0)
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "bool"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_on_main(&self) -> bool {
        self.firmware_on_main_raw()
    }
    
    /// Get raw value of firmware_on_main
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_on_main_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of firmware_on_main
    #[inline(always)]
    pub fn set_firmware_on_main(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// board_on_time_seconds
    ///
    /// time in seconds which the board has been powered on and running
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: "seconds"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn board_on_time_seconds(&self) -> u16 {
        self.board_on_time_seconds_raw()
    }
    
    /// Get raw value of board_on_time_seconds
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn board_on_time_seconds_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of board_on_time_seconds
    #[inline(always)]
    pub fn set_board_on_time_seconds(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 602 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// firmware_version
    ///
    /// short hash of the firmware
    ///
    /// - Min: 0
    /// - Max: 4294967295
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version_raw()
    }
    
    /// Get raw value of firmware_version
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn firmware_version_raw(&self) -> u32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        signal
    }
    
    /// Set value of firmware_version
    #[inline(always)]
    pub fn set_firmware_version(&mut self, value: u32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u32 || 4294967295_u32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 602 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for AcuBoardData {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for AcuBoardData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("AcuBoardData")
                .field("board_humidity", &self.board_humidity())
                .field("board_temperature", &self.board_temperature())
                .field("firmware_is_dirty", &self.firmware_is_dirty())
                .field("firmware_on_main", &self.firmware_on_main())
                .field("board_on_time_seconds", &self.board_on_time_seconds())
                .field("firmware_version", &self.firmware_version())
            .finish()
        } else {
            f.debug_tuple("AcuBoardData").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for AcuBoardData {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let board_humidity = u.int_in_range(0..=127)?;
        let board_temperature = u.int_in_range(0..=127)?;
        let firmware_is_dirty = u.int_in_range(0..=1)? == 1;
        let firmware_on_main = u.int_in_range(0..=1)? == 1;
        let board_on_time_seconds = u.int_in_range(0..=65535)?;
        let firmware_version = u.int_in_range(0..=4294967295)?;
        AcuBoardData::new(board_humidity,board_temperature,firmware_is_dirty,firmware_on_main,board_on_time_seconds,firmware_version).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_steeringpot
///
/// - ID: 899 (0x383)
/// - Size: 2 bytes
/// - Transmitter: cornernode_fl
///
/// mrow
#[derive(Clone, Copy)]
pub struct CornernodeSteeringpot {
    raw: [u8; 2],
}

impl CornernodeSteeringpot {
    pub const MESSAGE_ID: u32 = 899;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_steeringpot from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 899 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeSteeringpot {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeSteeringpot {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeSteeringpot")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeSteeringpot").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeSteeringpot {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeSteeringpot::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_fl_shockpot
///
/// - ID: 900 (0x384)
/// - Size: 2 bytes
/// - Transmitter: cornernode_fl
///
/// front left corner node shockpot data
#[derive(Clone, Copy)]
pub struct CornernodeFlShockpot {
    raw: [u8; 2],
}

impl CornernodeFlShockpot {
    pub const MESSAGE_ID: u32 = 900;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_fl_shockpot from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 900 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeFlShockpot {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeFlShockpot {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeFlShockpot")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeFlShockpot").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeFlShockpot {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeFlShockpot::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_fr_shockpot
///
/// - ID: 901 (0x385)
/// - Size: 2 bytes
/// - Transmitter: cornernode_fr
///
/// front right corner node shockpot data
#[derive(Clone, Copy)]
pub struct CornernodeFrShockpot {
    raw: [u8; 2],
}

impl CornernodeFrShockpot {
    pub const MESSAGE_ID: u32 = 901;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_fr_shockpot from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 901 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeFrShockpot {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeFrShockpot {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeFrShockpot")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeFrShockpot").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeFrShockpot {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeFrShockpot::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_rl_shockpot
///
/// - ID: 902 (0x386)
/// - Size: 2 bytes
/// - Transmitter: cornernode_rl
///
/// rear left corner node shockpot data
#[derive(Clone, Copy)]
pub struct CornernodeRlShockpot {
    raw: [u8; 2],
}

impl CornernodeRlShockpot {
    pub const MESSAGE_ID: u32 = 902;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_rl_shockpot from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 902 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeRlShockpot {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeRlShockpot {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeRlShockpot")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeRlShockpot").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeRlShockpot {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeRlShockpot::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_rr_shockpot
///
/// - ID: 903 (0x387)
/// - Size: 2 bytes
/// - Transmitter: cornernode_rr
///
/// rear right corner node shockpot data
#[derive(Clone, Copy)]
pub struct CornernodeRrShockpot {
    raw: [u8; 2],
}

impl CornernodeRrShockpot {
    pub const MESSAGE_ID: u32 = 903;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_rr_shockpot from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 903 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeRrShockpot {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeRrShockpot {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeRrShockpot")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeRrShockpot").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeRrShockpot {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeRrShockpot::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_fl_wheelspeed
///
/// - ID: 904 (0x388)
/// - Size: 2 bytes
/// - Transmitter: cornernode_fl
///
/// front left corner node wheelspeed data
#[derive(Clone, Copy)]
pub struct CornernodeFlWheelspeed {
    raw: [u8; 2],
}

impl CornernodeFlWheelspeed {
    pub const MESSAGE_ID: u32 = 904;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_fl_wheelspeed from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 904 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeFlWheelspeed {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeFlWheelspeed {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeFlWheelspeed")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeFlWheelspeed").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeFlWheelspeed {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeFlWheelspeed::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_fr_wheelspeed
///
/// - ID: 905 (0x389)
/// - Size: 2 bytes
/// - Transmitter: cornernode_fr
///
/// front right corner node wheelspeed data
#[derive(Clone, Copy)]
pub struct CornernodeFrWheelspeed {
    raw: [u8; 2],
}

impl CornernodeFrWheelspeed {
    pub const MESSAGE_ID: u32 = 905;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_fr_wheelspeed from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 905 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeFrWheelspeed {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeFrWheelspeed {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeFrWheelspeed")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeFrWheelspeed").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeFrWheelspeed {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeFrWheelspeed::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_rl_wheelspeed
///
/// - ID: 906 (0x38a)
/// - Size: 2 bytes
/// - Transmitter: cornernode_rl
///
/// rear left corner node wheelspeed data
#[derive(Clone, Copy)]
pub struct CornernodeRlWheelspeed {
    raw: [u8; 2],
}

impl CornernodeRlWheelspeed {
    pub const MESSAGE_ID: u32 = 906;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_rl_wheelspeed from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 906 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeRlWheelspeed {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeRlWheelspeed {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeRlWheelspeed")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeRlWheelspeed").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeRlWheelspeed {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeRlWheelspeed::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_rr_wheelspeed
///
/// - ID: 907 (0x38b)
/// - Size: 2 bytes
/// - Transmitter: cornernode_rr
///
/// rear right corner node wheelspeed data
#[derive(Clone, Copy)]
pub struct CornernodeRrWheelspeed {
    raw: [u8; 2],
}

impl CornernodeRrWheelspeed {
    pub const MESSAGE_ID: u32 = 907;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_rr_wheelspeed from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 907 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeRrWheelspeed {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeRrWheelspeed {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeRrWheelspeed")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeRrWheelspeed").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeRrWheelspeed {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeRrWheelspeed::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_fl_tiretemp
///
/// - ID: 908 (0x38c)
/// - Size: 8 bytes
/// - Transmitter: cornernode_fl
///
/// front left corner node tiretemp data
#[derive(Clone, Copy)]
pub struct CornernodeFlTiretemp {
    raw: [u8; 8],
}

impl CornernodeFlTiretemp {
    pub const MESSAGE_ID: u32 = 908;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_fl_tiretemp from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 908 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeFlTiretemp {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeFlTiretemp {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeFlTiretemp")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeFlTiretemp").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeFlTiretemp {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeFlTiretemp::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_fr_tiretemp
///
/// - ID: 909 (0x38d)
/// - Size: 8 bytes
/// - Transmitter: cornernode_fr
///
/// front right corner node tiretemp data
#[derive(Clone, Copy)]
pub struct CornernodeFrTiretemp {
    raw: [u8; 8],
}

impl CornernodeFrTiretemp {
    pub const MESSAGE_ID: u32 = 909;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_fr_tiretemp from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 909 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeFrTiretemp {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeFrTiretemp {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeFrTiretemp")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeFrTiretemp").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeFrTiretemp {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeFrTiretemp::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_rl_tiretemp
///
/// - ID: 910 (0x38e)
/// - Size: 8 bytes
/// - Transmitter: cornernode_rl
///
/// rear left corner node tiretemp data
#[derive(Clone, Copy)]
pub struct CornernodeRlTiretemp {
    raw: [u8; 8],
}

impl CornernodeRlTiretemp {
    pub const MESSAGE_ID: u32 = 910;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_rl_tiretemp from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 910 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeRlTiretemp {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeRlTiretemp {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeRlTiretemp")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeRlTiretemp").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeRlTiretemp {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeRlTiretemp::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_rr_tiretemp
///
/// - ID: 911 (0x38f)
/// - Size: 8 bytes
/// - Transmitter: cornernode_rr
///
/// rear right corner node tiretemp data
#[derive(Clone, Copy)]
pub struct CornernodeRrTiretemp {
    raw: [u8; 8],
}

impl CornernodeRrTiretemp {
    pub const MESSAGE_ID: u32 = 911;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_rr_tiretemp from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 911 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeRrTiretemp {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeRrTiretemp {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeRrTiretemp")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeRrTiretemp").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeRrTiretemp {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeRrTiretemp::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_front_brakepressure
///
/// - ID: 912 (0x390)
/// - Size: 2 bytes
/// - Transmitter: cornernode_fl
///
/// front brakepressure data
#[derive(Clone, Copy)]
pub struct CornernodeFrontBrakepressure {
    raw: [u8; 2],
}

impl CornernodeFrontBrakepressure {
    pub const MESSAGE_ID: u32 = 912;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_front_brakepressure from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 912 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeFrontBrakepressure {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeFrontBrakepressure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeFrontBrakepressure")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeFrontBrakepressure").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeFrontBrakepressure {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeFrontBrakepressure::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_rear_brakepressure
///
/// - ID: 913 (0x391)
/// - Size: 2 bytes
/// - Transmitter: cornernode_fr
///
/// front right corner node brakepressure data
#[derive(Clone, Copy)]
pub struct CornernodeRearBrakepressure {
    raw: [u8; 2],
}

impl CornernodeRearBrakepressure {
    pub const MESSAGE_ID: u32 = 913;
    
    pub const UINT16_MIN: u16 = 0_u16;
    pub const UINT16_MAX: u16 = 65535_u16;
    
    /// Construct new cornernode_rear_brakepressure from values
    pub fn new(uint16: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_uint16(uint16)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// uint16
    ///
    /// Raw unsigned integer with 16 bits
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn uint16(&self) -> u16 {
        self.uint16_raw()
    }
    
    /// Get raw value of uint16
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn uint16_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of uint16
    #[inline(always)]
    pub fn set_uint16(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 913 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeRearBrakepressure {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeRearBrakepressure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeRearBrakepressure")
                .field("uint16", &self.uint16())
            .finish()
        } else {
            f.debug_tuple("CornernodeRearBrakepressure").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeRearBrakepressure {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let uint16 = u.int_in_range(0..=65535)?;
        CornernodeRearBrakepressure::new(uint16).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_motor_temp
///
/// - ID: 914 (0x392)
/// - Size: 2 bytes
/// - Transmitter: cornernode_rr
///
/// The motor thermistor fella
#[derive(Clone, Copy)]
pub struct CornernodeMotorTemp {
    raw: [u8; 2],
}

impl CornernodeMotorTemp {
    pub const MESSAGE_ID: u32 = 914;
    
    pub const TEMP_C_MIN: f32 = 0_f32;
    pub const TEMP_C_MAX: f32 = 65535_f32;
    
    /// Construct new cornernode_motor_temp from values
    pub fn new(temp_c: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_temp_c(temp_c)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// temp_C
    ///
    /// Temperature in degrees celcius
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn temp_c(&self) -> f32 {
        self.temp_c_raw()
    }
    
    /// Get raw value of temp_C
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn temp_c_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of temp_C
    #[inline(always)]
    pub fn set_temp_c(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 914 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeMotorTemp {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeMotorTemp {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeMotorTemp")
                .field("temp_c", &self.temp_c())
            .finish()
        } else {
            f.debug_tuple("CornernodeMotorTemp").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeMotorTemp {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let temp_c = u.float_in_range(0_f32..=65535_f32)?;
        CornernodeMotorTemp::new(temp_c).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// cornernode_coolant_temp
///
/// - ID: 915 (0x393)
/// - Size: 2 bytes
/// - Transmitter: cornernode_rr
///
/// The coolant thermistor fella
#[derive(Clone, Copy)]
pub struct CornernodeCoolantTemp {
    raw: [u8; 2],
}

impl CornernodeCoolantTemp {
    pub const MESSAGE_ID: u32 = 915;
    
    pub const TEMP_C_MIN: f32 = 0_f32;
    pub const TEMP_C_MAX: f32 = 65535_f32;
    
    /// Construct new cornernode_coolant_temp from values
    pub fn new(temp_c: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_temp_c(temp_c)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// temp_C
    ///
    /// Temperature in degrees celcius
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn temp_c(&self) -> f32 {
        self.temp_c_raw()
    }
    
    /// Get raw value of temp_C
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn temp_c_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of temp_C
    #[inline(always)]
    pub fn set_temp_c(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 65535_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 915 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CornernodeCoolantTemp {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CornernodeCoolantTemp {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CornernodeCoolantTemp")
                .field("temp_c", &self.temp_c())
            .finish()
        } else {
            f.debug_tuple("CornernodeCoolantTemp").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CornernodeCoolantTemp {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let temp_c = u.float_in_range(0_f32..=65535_f32)?;
        CornernodeCoolantTemp::new(temp_c).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// MSGID_0X6B1
///
/// - ID: 1713 (0x6b1)
/// - Size: 8 bytes
/// - Transmitter: BMS
///
/// This ID Transmits at 8 ms.
#[derive(Clone, Copy)]
pub struct Msgid0x6b1 {
    raw: [u8; 8],
}

impl Msgid0x6b1 {
    pub const MESSAGE_ID: u32 = 1713;
    
    pub const PACK_SUMMED_VOLTAGE_MIN: f32 = 0_f32;
    pub const PACK_SUMMED_VOLTAGE_MAX: f32 = 0_f32;
    pub const PACK_OPEN_VOLTAGE_MIN: f32 = 0_f32;
    pub const PACK_OPEN_VOLTAGE_MAX: f32 = 0_f32;
    pub const PACK_INST_VOLTAGE_MIN: f32 = 0_f32;
    pub const PACK_INST_VOLTAGE_MAX: f32 = 0_f32;
    pub const PACK_CURRENT_MIN: f32 = 0_f32;
    pub const PACK_CURRENT_MAX: f32 = 0_f32;
    
    /// Construct new MSGID_0X6B1 from values
    pub fn new(pack_summed_voltage: f32, pack_open_voltage: f32, pack_inst_voltage: f32, pack_current: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_pack_summed_voltage(pack_summed_voltage)?;
        res.set_pack_open_voltage(pack_open_voltage)?;
        res.set_pack_inst_voltage(pack_inst_voltage)?;
        res.set_pack_current(pack_current)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// Pack_Summed_Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Volts"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn pack_summed_voltage(&self) -> f32 {
        self.pack_summed_voltage_raw()
    }
    
    /// Get raw value of Pack_Summed_Voltage
    ///
    /// - Start bit: 55
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pack_summed_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[48..64].load_be::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Pack_Summed_Voltage
    #[inline(always)]
    pub fn set_pack_summed_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1713 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[48..64].store_be(value);
        Ok(())
    }
    
    /// Pack_Open_Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Volts"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn pack_open_voltage(&self) -> f32 {
        self.pack_open_voltage_raw()
    }
    
    /// Get raw value of Pack_Open_Voltage
    ///
    /// - Start bit: 39
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pack_open_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[32..48].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Pack_Open_Voltage
    #[inline(always)]
    pub fn set_pack_open_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1713 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[32..48].store_be(value);
        Ok(())
    }
    
    /// Pack_Inst_Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Volts"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn pack_inst_voltage(&self) -> f32 {
        self.pack_inst_voltage_raw()
    }
    
    /// Get raw value of Pack_Inst_Voltage
    ///
    /// - Start bit: 23
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pack_inst_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[16..32].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Pack_Inst_Voltage
    #[inline(always)]
    pub fn set_pack_inst_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1713 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[16..32].store_be(value);
        Ok(())
    }
    
    /// Pack_Current
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Amps"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn pack_current(&self) -> f32 {
        self.pack_current_raw()
    }
    
    /// Get raw value of Pack_Current
    ///
    /// - Start bit: 7
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn pack_current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[0..16].load_be::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Pack_Current
    #[inline(always)]
    pub fn set_pack_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1713 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Msb0>()[0..16].store_be(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Msgid0x6b1 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Msgid0x6b1 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Msgid0x6b1")
                .field("pack_summed_voltage", &self.pack_summed_voltage())
                .field("pack_open_voltage", &self.pack_open_voltage())
                .field("pack_inst_voltage", &self.pack_inst_voltage())
                .field("pack_current", &self.pack_current())
            .finish()
        } else {
            f.debug_tuple("Msgid0x6b1").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Msgid0x6b1 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let pack_summed_voltage = u.float_in_range(0_f32..=0_f32)?;
        let pack_open_voltage = u.float_in_range(0_f32..=0_f32)?;
        let pack_inst_voltage = u.float_in_range(0_f32..=0_f32)?;
        let pack_current = u.float_in_range(0_f32..=0_f32)?;
        Msgid0x6b1::new(pack_summed_voltage,pack_open_voltage,pack_inst_voltage,pack_current).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// MSGID_0X6B2
///
/// - ID: 1714 (0x6b2)
/// - Size: 7 bytes
/// - Transmitter: BMS
///
/// This ID Transmits at 8 ms.
#[derive(Clone, Copy)]
pub struct Msgid0x6b2 {
    raw: [u8; 7],
}

impl Msgid0x6b2 {
    pub const MESSAGE_ID: u32 = 1714;
    
    pub const LOW_CELL_VOLTAGE_MIN: f32 = 0_f32;
    pub const LOW_CELL_VOLTAGE_MAX: f32 = 0_f32;
    pub const AVG_CELL_VOLTAGE_MIN: f32 = 0_f32;
    pub const AVG_CELL_VOLTAGE_MAX: f32 = 0_f32;
    pub const AVERAGE_TEMPERATURE_MIN: u8 = 0_u8;
    pub const AVERAGE_TEMPERATURE_MAX: u8 = 0_u8;
    pub const LOW_TEMPERATURE_MIN: u8 = 0_u8;
    pub const LOW_TEMPERATURE_MAX: u8 = 0_u8;
    pub const HIGH_TEMPERATURE_MIN: u8 = 0_u8;
    pub const HIGH_TEMPERATURE_MAX: u8 = 0_u8;
    
    /// Construct new MSGID_0X6B2 from values
    pub fn new(low_cell_voltage: f32, avg_cell_voltage: f32, average_temperature: u8, low_temperature: u8, high_temperature: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 7] };
        res.set_low_cell_voltage(low_cell_voltage)?;
        res.set_avg_cell_voltage(avg_cell_voltage)?;
        res.set_average_temperature(average_temperature)?;
        res.set_low_temperature(low_temperature)?;
        res.set_high_temperature(high_temperature)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 7] {
        &self.raw
    }
    
    /// Low_Cell_Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Volts"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn low_cell_voltage(&self) -> f32 {
        self.low_cell_voltage_raw()
    }
    
    /// Get raw value of Low_Cell_Voltage
    ///
    /// - Start bit: 47
    /// - Signal size: 16 bits
    /// - Factor: 0.0001
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn low_cell_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[40..56].load_be::<u16>();
        
        let factor = 0.0001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Low_Cell_Voltage
    #[inline(always)]
    pub fn set_low_cell_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1714 });
        }
        let factor = 0.0001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[40..56].store_be(value);
        Ok(())
    }
    
    /// Avg_Cell_Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Volts"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn avg_cell_voltage(&self) -> f32 {
        self.avg_cell_voltage_raw()
    }
    
    /// Get raw value of Avg_Cell_Voltage
    ///
    /// - Start bit: 31
    /// - Signal size: 16 bits
    /// - Factor: 0.0001
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn avg_cell_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[24..40].load_be::<u16>();
        
        let factor = 0.0001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Avg_Cell_Voltage
    #[inline(always)]
    pub fn set_avg_cell_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1714 });
        }
        let factor = 0.0001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[24..40].store_be(value);
        Ok(())
    }
    
    /// Average_Temperature
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Celsius"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn average_temperature(&self) -> u8 {
        self.average_temperature_raw()
    }
    
    /// Get raw value of Average_Temperature
    ///
    /// - Start bit: 23
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn average_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Msb0>()[16..24].load_be::<u8>();
        
        signal
    }
    
    /// Set value of Average_Temperature
    #[inline(always)]
    pub fn set_average_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 0_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1714 });
        }
        self.raw.view_bits_mut::<Msb0>()[16..24].store_be(value);
        Ok(())
    }
    
    /// Low_Temperature
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Celsius"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn low_temperature(&self) -> u8 {
        self.low_temperature_raw()
    }
    
    /// Get raw value of Low_Temperature
    ///
    /// - Start bit: 15
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn low_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Msb0>()[8..16].load_be::<u8>();
        
        signal
    }
    
    /// Set value of Low_Temperature
    #[inline(always)]
    pub fn set_low_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 0_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1714 });
        }
        self.raw.view_bits_mut::<Msb0>()[8..16].store_be(value);
        Ok(())
    }
    
    /// High_Temperature
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Celsius"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn high_temperature(&self) -> u8 {
        self.high_temperature_raw()
    }
    
    /// Get raw value of High_Temperature
    ///
    /// - Start bit: 7
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn high_temperature_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Msb0>()[0..8].load_be::<u8>();
        
        signal
    }
    
    /// Set value of High_Temperature
    #[inline(always)]
    pub fn set_high_temperature(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 0_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1714 });
        }
        self.raw.view_bits_mut::<Msb0>()[0..8].store_be(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Msgid0x6b2 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 7 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&payload[..7]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Msgid0x6b2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Msgid0x6b2")
                .field("low_cell_voltage", &self.low_cell_voltage())
                .field("avg_cell_voltage", &self.avg_cell_voltage())
                .field("average_temperature", &self.average_temperature())
                .field("low_temperature", &self.low_temperature())
                .field("high_temperature", &self.high_temperature())
            .finish()
        } else {
            f.debug_tuple("Msgid0x6b2").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Msgid0x6b2 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let low_cell_voltage = u.float_in_range(0_f32..=0_f32)?;
        let avg_cell_voltage = u.float_in_range(0_f32..=0_f32)?;
        let average_temperature = u.int_in_range(0..=0)?;
        let low_temperature = u.int_in_range(0..=0)?;
        let high_temperature = u.int_in_range(0..=0)?;
        Msgid0x6b2::new(low_cell_voltage,avg_cell_voltage,average_temperature,low_temperature,high_temperature).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// MSGID_0X6B3
///
/// - ID: 1715 (0x6b3)
/// - Size: 1 bytes
/// - Transmitter: BMS
///
/// This ID Transmits at 104 ms.
#[derive(Clone, Copy)]
pub struct Msgid0x6b3 {
    raw: [u8; 1],
}

impl Msgid0x6b3 {
    pub const MESSAGE_ID: u32 = 1715;
    
    pub const PACK_SOC_MIN: f32 = 0_f32;
    pub const PACK_SOC_MAX: f32 = 0_f32;
    
    /// Construct new MSGID_0X6B3 from values
    pub fn new(pack_soc: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_pack_soc(pack_soc)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// Pack_SOC
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Percent"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn pack_soc(&self) -> f32 {
        self.pack_soc_raw()
    }
    
    /// Get raw value of Pack_SOC
    ///
    /// - Start bit: 7
    /// - Signal size: 8 bits
    /// - Factor: 0.5
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pack_soc_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[0..8].load_be::<u8>();
        
        let factor = 0.5_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Pack_SOC
    #[inline(always)]
    pub fn set_pack_soc(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 1715 });
        }
        let factor = 0.5_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Msb0>()[0..8].store_be(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Msgid0x6b3 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Msgid0x6b3 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Msgid0x6b3")
                .field("pack_soc", &self.pack_soc())
            .finish()
        } else {
            f.debug_tuple("Msgid0x6b3").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Msgid0x6b3 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let pack_soc = u.float_in_range(0_f32..=0_f32)?;
        Msgid0x6b3::new(pack_soc).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// MSGID_0X6B4
///
/// - ID: 1716 (0x6b4)
/// - Size: 1 bytes
/// - Transmitter: BMS
///
/// This ID Transmits at 104 ms.
#[derive(Clone, Copy)]
pub struct Msgid0x6b4 {
    raw: [u8; 1],
}

impl Msgid0x6b4 {
    pub const MESSAGE_ID: u32 = 1716;
    
    
    /// Construct new MSGID_0X6B4 from values
    pub fn new(dtc_p0aa6_high_voltage_isolation: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_dtc_p0aa6_high_voltage_isolation(dtc_p0aa6_high_voltage_isolation)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// DTC_P0AA6_High_Voltage_Isolation
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn dtc_p0aa6_high_voltage_isolation(&self) -> bool {
        self.dtc_p0aa6_high_voltage_isolation_raw()
    }
    
    /// Get raw value of DTC_P0AA6_High_Voltage_Isolation
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dtc_p0aa6_high_voltage_isolation_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of DTC_P0AA6_High_Voltage_Isolation
    #[inline(always)]
    pub fn set_dtc_p0aa6_high_voltage_isolation(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Msgid0x6b4 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Msgid0x6b4 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Msgid0x6b4")
                .field("dtc_p0aa6_high_voltage_isolation", &self.dtc_p0aa6_high_voltage_isolation())
            .finish()
        } else {
            f.debug_tuple("Msgid0x6b4").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Msgid0x6b4 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let dtc_p0aa6_high_voltage_isolation = u.int_in_range(0..=1)? == 1;
        Msgid0x6b4::new(dtc_p0aa6_high_voltage_isolation).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// MSGID_0X1806E7F4
///
/// - ID: 2550589428 (0x9806e7f4)
/// - Size: 8 bytes
/// - Transmitter: BMS
///
/// This ID Transmits at 808 ms.
#[derive(Clone, Copy)]
pub struct Msgid0x1806e7f4 {
    raw: [u8; 8],
}

impl Msgid0x1806e7f4 {
    pub const MESSAGE_ID: u32 = 2550589428;
    
    pub const PACK_CCL_MIN: f32 = 0_f32;
    pub const PACK_CCL_MAX: f32 = 0_f32;
    pub const MAXIMUM_PACK_VOLTAGE_MIN: f32 = 0_f32;
    pub const MAXIMUM_PACK_VOLTAGE_MAX: f32 = 0_f32;
    
    /// Construct new MSGID_0X1806E7F4 from values
    pub fn new(dtc_p0a08_charger_safety_relay_f: bool, pack_ccl: f32, maximum_pack_voltage: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_dtc_p0a08_charger_safety_relay_f(dtc_p0a08_charger_safety_relay_f)?;
        res.set_pack_ccl(pack_ccl)?;
        res.set_maximum_pack_voltage(maximum_pack_voltage)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// DTC_P0A08_Charger_Safety_Relay_F
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn dtc_p0a08_charger_safety_relay_f(&self) -> bool {
        self.dtc_p0a08_charger_safety_relay_f_raw()
    }
    
    /// Get raw value of DTC_P0A08_Charger_Safety_Relay_F
    ///
    /// - Start bit: 39
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dtc_p0a08_charger_safety_relay_f_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[39..40].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of DTC_P0A08_Charger_Safety_Relay_F
    #[inline(always)]
    pub fn set_dtc_p0a08_charger_safety_relay_f(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[39..40].store_le(value);
        Ok(())
    }
    
    /// Pack_CCL
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Amps"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn pack_ccl(&self) -> f32 {
        self.pack_ccl_raw()
    }
    
    /// Get raw value of Pack_CCL
    ///
    /// - Start bit: 23
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pack_ccl_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[16..32].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Pack_CCL
    #[inline(always)]
    pub fn set_pack_ccl(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 2550589428 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[16..32].store_be(value);
        Ok(())
    }
    
    /// Maximum_Pack_Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Volts"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn maximum_pack_voltage(&self) -> f32 {
        self.maximum_pack_voltage_raw()
    }
    
    /// Get raw value of Maximum_Pack_Voltage
    ///
    /// - Start bit: 7
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn maximum_pack_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[0..16].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Maximum_Pack_Voltage
    #[inline(always)]
    pub fn set_maximum_pack_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 2550589428 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[0..16].store_be(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Msgid0x1806e7f4 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Msgid0x1806e7f4 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Msgid0x1806e7f4")
                .field("dtc_p0a08_charger_safety_relay_f", &self.dtc_p0a08_charger_safety_relay_f())
                .field("pack_ccl", &self.pack_ccl())
                .field("maximum_pack_voltage", &self.maximum_pack_voltage())
            .finish()
        } else {
            f.debug_tuple("Msgid0x1806e7f4").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Msgid0x1806e7f4 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let dtc_p0a08_charger_safety_relay_f = u.int_in_range(0..=1)? == 1;
        let pack_ccl = u.float_in_range(0_f32..=0_f32)?;
        let maximum_pack_voltage = u.float_in_range(0_f32..=0_f32)?;
        Msgid0x1806e7f4::new(dtc_p0a08_charger_safety_relay_f,pack_ccl,maximum_pack_voltage).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// MSGID_0X1806E5F4
///
/// - ID: 2550588916 (0x9806e5f4)
/// - Size: 8 bytes
/// - Transmitter: BMS
///
/// This ID Transmits at 808 ms.
#[derive(Clone, Copy)]
pub struct Msgid0x1806e5f4 {
    raw: [u8; 8],
}

impl Msgid0x1806e5f4 {
    pub const MESSAGE_ID: u32 = 2550588916;
    
    pub const PACK_CCL_MIN: f32 = 0_f32;
    pub const PACK_CCL_MAX: f32 = 0_f32;
    pub const MAXIMUM_PACK_VOLTAGE_MIN: f32 = 0_f32;
    pub const MAXIMUM_PACK_VOLTAGE_MAX: f32 = 0_f32;
    
    /// Construct new MSGID_0X1806E5F4 from values
    pub fn new(dtc_p0a08_charger_safety_re_0000: bool, pack_ccl: f32, maximum_pack_voltage: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_dtc_p0a08_charger_safety_re_0000(dtc_p0a08_charger_safety_re_0000)?;
        res.set_pack_ccl(pack_ccl)?;
        res.set_maximum_pack_voltage(maximum_pack_voltage)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// DTC_P0A08_Charger_Safety_Re_0000
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn dtc_p0a08_charger_safety_re_0000(&self) -> bool {
        self.dtc_p0a08_charger_safety_re_0000_raw()
    }
    
    /// Get raw value of DTC_P0A08_Charger_Safety_Re_0000
    ///
    /// - Start bit: 39
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dtc_p0a08_charger_safety_re_0000_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[39..40].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of DTC_P0A08_Charger_Safety_Re_0000
    #[inline(always)]
    pub fn set_dtc_p0a08_charger_safety_re_0000(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[39..40].store_le(value);
        Ok(())
    }
    
    /// Pack_CCL
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Amps"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn pack_ccl(&self) -> f32 {
        self.pack_ccl_raw()
    }
    
    /// Get raw value of Pack_CCL
    ///
    /// - Start bit: 23
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pack_ccl_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[16..32].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Pack_CCL
    #[inline(always)]
    pub fn set_pack_ccl(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 2550588916 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[16..32].store_be(value);
        Ok(())
    }
    
    /// Maximum_Pack_Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Volts"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn maximum_pack_voltage(&self) -> f32 {
        self.maximum_pack_voltage_raw()
    }
    
    /// Get raw value of Maximum_Pack_Voltage
    ///
    /// - Start bit: 7
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn maximum_pack_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[0..16].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Maximum_Pack_Voltage
    #[inline(always)]
    pub fn set_maximum_pack_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 2550588916 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[0..16].store_be(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Msgid0x1806e5f4 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Msgid0x1806e5f4 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Msgid0x1806e5f4")
                .field("dtc_p0a08_charger_safety_re_0000", &self.dtc_p0a08_charger_safety_re_0000())
                .field("pack_ccl", &self.pack_ccl())
                .field("maximum_pack_voltage", &self.maximum_pack_voltage())
            .finish()
        } else {
            f.debug_tuple("Msgid0x1806e5f4").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Msgid0x1806e5f4 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let dtc_p0a08_charger_safety_re_0000 = u.int_in_range(0..=1)? == 1;
        let pack_ccl = u.float_in_range(0_f32..=0_f32)?;
        let maximum_pack_voltage = u.float_in_range(0_f32..=0_f32)?;
        Msgid0x1806e5f4::new(dtc_p0a08_charger_safety_re_0000,pack_ccl,maximum_pack_voltage).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// MSGID_0X1806E9F4
///
/// - ID: 2550589940 (0x9806e9f4)
/// - Size: 8 bytes
/// - Transmitter: BMS
///
/// This ID Transmits at 808 ms.
#[derive(Clone, Copy)]
pub struct Msgid0x1806e9f4 {
    raw: [u8; 8],
}

impl Msgid0x1806e9f4 {
    pub const MESSAGE_ID: u32 = 2550589940;
    
    pub const PACK_CCL_MIN: f32 = 0_f32;
    pub const PACK_CCL_MAX: f32 = 0_f32;
    pub const MAXIMUM_PACK_VOLTAGE_MIN: f32 = 0_f32;
    pub const MAXIMUM_PACK_VOLTAGE_MAX: f32 = 0_f32;
    
    /// Construct new MSGID_0X1806E9F4 from values
    pub fn new(dtc_p0a08_charger_safety_re_0001: bool, pack_ccl: f32, maximum_pack_voltage: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_dtc_p0a08_charger_safety_re_0001(dtc_p0a08_charger_safety_re_0001)?;
        res.set_pack_ccl(pack_ccl)?;
        res.set_maximum_pack_voltage(maximum_pack_voltage)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// DTC_P0A08_Charger_Safety_Re_0001
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn dtc_p0a08_charger_safety_re_0001(&self) -> bool {
        self.dtc_p0a08_charger_safety_re_0001_raw()
    }
    
    /// Get raw value of DTC_P0A08_Charger_Safety_Re_0001
    ///
    /// - Start bit: 39
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn dtc_p0a08_charger_safety_re_0001_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[39..40].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of DTC_P0A08_Charger_Safety_Re_0001
    #[inline(always)]
    pub fn set_dtc_p0a08_charger_safety_re_0001(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[39..40].store_le(value);
        Ok(())
    }
    
    /// Pack_CCL
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Amps"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn pack_ccl(&self) -> f32 {
        self.pack_ccl_raw()
    }
    
    /// Get raw value of Pack_CCL
    ///
    /// - Start bit: 23
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pack_ccl_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[16..32].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Pack_CCL
    #[inline(always)]
    pub fn set_pack_ccl(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 2550589940 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[16..32].store_be(value);
        Ok(())
    }
    
    /// Maximum_Pack_Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Volts"
    /// - Receivers: Third_Party_Device
    #[inline(always)]
    pub fn maximum_pack_voltage(&self) -> f32 {
        self.maximum_pack_voltage_raw()
    }
    
    /// Get raw value of Maximum_Pack_Voltage
    ///
    /// - Start bit: 7
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn maximum_pack_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[0..16].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of Maximum_Pack_Voltage
    #[inline(always)]
    pub fn set_maximum_pack_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 2550589940 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[0..16].store_be(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Msgid0x1806e9f4 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Msgid0x1806e9f4 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Msgid0x1806e9f4")
                .field("dtc_p0a08_charger_safety_re_0001", &self.dtc_p0a08_charger_safety_re_0001())
                .field("pack_ccl", &self.pack_ccl())
                .field("maximum_pack_voltage", &self.maximum_pack_voltage())
            .finish()
        } else {
            f.debug_tuple("Msgid0x1806e9f4").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Msgid0x1806e9f4 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let dtc_p0a08_charger_safety_re_0001 = u.int_in_range(0..=1)? == 1;
        let pack_ccl = u.float_in_range(0_f32..=0_f32)?;
        let maximum_pack_voltage = u.float_in_range(0_f32..=0_f32)?;
        Msgid0x1806e9f4::new(dtc_p0a08_charger_safety_re_0001,pack_ccl,maximum_pack_voltage).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M173_Modulation_And_Flux_Info
///
/// - ID: 173 (0xad)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M173ModulationAndFluxInfo {
    raw: [u8; 8],
}

impl M173ModulationAndFluxInfo {
    pub const MESSAGE_ID: u32 = 173;
    
    pub const INV_IQ_COMMAND_MIN: f32 = -3276.8_f32;
    pub const INV_IQ_COMMAND_MAX: f32 = 3276.7_f32;
    pub const INV_ID_COMMAND_MIN: f32 = -3276.8_f32;
    pub const INV_ID_COMMAND_MAX: f32 = 3276.7_f32;
    pub const INV_FLUX_WEAKENING_OUTPUT_MIN: f32 = -3276.8_f32;
    pub const INV_FLUX_WEAKENING_OUTPUT_MAX: f32 = 3276.7_f32;
    pub const INV_MODULATION_INDEX_MIN: f32 = -3.2768_f32;
    pub const INV_MODULATION_INDEX_MAX: f32 = 3.2767_f32;
    
    /// Construct new M173_Modulation_And_Flux_Info from values
    pub fn new(inv_iq_command: f32, inv_id_command: f32, inv_flux_weakening_output: f32, inv_modulation_index: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_iq_command(inv_iq_command)?;
        res.set_inv_id_command(inv_id_command)?;
        res.set_inv_flux_weakening_output(inv_flux_weakening_output)?;
        res.set_inv_modulation_index(inv_modulation_index)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Iq_Command
    ///
    /// The commanded Q-axis current
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_iq_command(&self) -> f32 {
        self.inv_iq_command_raw()
    }
    
    /// Get raw value of INV_Iq_Command
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_iq_command_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Iq_Command
    #[inline(always)]
    pub fn set_inv_iq_command(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 173 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Id_Command
    ///
    /// The commanded D-axis current
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_id_command(&self) -> f32 {
        self.inv_id_command_raw()
    }
    
    /// Get raw value of INV_Id_Command
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_id_command_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Id_Command
    #[inline(always)]
    pub fn set_inv_id_command(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 173 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Flux_Weakening_Output
    ///
    /// This is the current output of the flux regulator.
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_flux_weakening_output(&self) -> f32 {
        self.inv_flux_weakening_output_raw()
    }
    
    /// Get raw value of INV_Flux_Weakening_Output
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_flux_weakening_output_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Flux_Weakening_Output
    #[inline(always)]
    pub fn set_inv_flux_weakening_output(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 173 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Modulation_Index
    ///
    /// This is the modulation index. The scale factor is x100. To get the actual modulation index divide the value by 100.
    ///
    /// - Min: -3.2768
    /// - Max: 3.2767
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_modulation_index(&self) -> f32 {
        self.inv_modulation_index_raw()
    }
    
    /// Get raw value of INV_Modulation_Index
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.0001
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_modulation_index_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.0001_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Modulation_Index
    #[inline(always)]
    pub fn set_inv_modulation_index(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3.2768_f32 || 3.2767_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 173 });
        }
        let factor = 0.0001_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M173ModulationAndFluxInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M173ModulationAndFluxInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M173ModulationAndFluxInfo")
                .field("inv_iq_command", &self.inv_iq_command())
                .field("inv_id_command", &self.inv_id_command())
                .field("inv_flux_weakening_output", &self.inv_flux_weakening_output())
                .field("inv_modulation_index", &self.inv_modulation_index())
            .finish()
        } else {
            f.debug_tuple("M173ModulationAndFluxInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M173ModulationAndFluxInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_iq_command = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_id_command = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_flux_weakening_output = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_modulation_index = u.float_in_range(-3.2768_f32..=3.2767_f32)?;
        M173ModulationAndFluxInfo::new(inv_iq_command,inv_id_command,inv_flux_weakening_output,inv_modulation_index).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M172_Torque_And_Timer_Info
///
/// - ID: 172 (0xac)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M172TorqueAndTimerInfo {
    raw: [u8; 8],
}

impl M172TorqueAndTimerInfo {
    pub const MESSAGE_ID: u32 = 172;
    
    pub const INV_POWER_ON_TIMER_MIN: f32 = 0_f32;
    pub const INV_POWER_ON_TIMER_MAX: f32 = 12884800_f32;
    pub const INV_TORQUE_FEEDBACK_MIN: f32 = -3276.8_f32;
    pub const INV_TORQUE_FEEDBACK_MAX: f32 = 3276.7_f32;
    pub const INV_COMMANDED_TORQUE_MIN: f32 = -3276.8_f32;
    pub const INV_COMMANDED_TORQUE_MAX: f32 = 3276.7_f32;
    
    /// Construct new M172_Torque_And_Timer_Info from values
    pub fn new(inv_power_on_timer: f32, inv_torque_feedback: f32, inv_commanded_torque: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_power_on_timer(inv_power_on_timer)?;
        res.set_inv_torque_feedback(inv_torque_feedback)?;
        res.set_inv_commanded_torque(inv_commanded_torque)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Power_On_Timer
    ///
    /// Updated every 3 msec. This will roll over in approximately 150 days!
    ///
    /// - Min: 0
    /// - Max: 12884800
    /// - Unit: "time:second"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_power_on_timer(&self) -> f32 {
        self.inv_power_on_timer_raw()
    }
    
    /// Get raw value of INV_Power_On_Timer
    ///
    /// - Start bit: 32
    /// - Signal size: 32 bits
    /// - Factor: 0.003
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_power_on_timer_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..64].load_le::<u32>();
        
        let factor = 0.003_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Power_On_Timer
    #[inline(always)]
    pub fn set_inv_power_on_timer(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 12884800_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 172 });
        }
        let factor = 0.003_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u32;
        
        self.raw.view_bits_mut::<Lsb0>()[32..64].store_le(value);
        Ok(())
    }
    
    /// INV_Torque_Feedback
    ///
    /// Estimated motor torque feedback
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_torque_feedback(&self) -> f32 {
        self.inv_torque_feedback_raw()
    }
    
    /// Get raw value of INV_Torque_Feedback
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_torque_feedback_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Torque_Feedback
    #[inline(always)]
    pub fn set_inv_torque_feedback(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 172 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Commanded_Torque
    ///
    /// The commanded Torque
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_commanded_torque(&self) -> f32 {
        self.inv_commanded_torque_raw()
    }
    
    /// Get raw value of INV_Commanded_Torque
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_commanded_torque_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Commanded_Torque
    #[inline(always)]
    pub fn set_inv_commanded_torque(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 172 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M172TorqueAndTimerInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M172TorqueAndTimerInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M172TorqueAndTimerInfo")
                .field("inv_power_on_timer", &self.inv_power_on_timer())
                .field("inv_torque_feedback", &self.inv_torque_feedback())
                .field("inv_commanded_torque", &self.inv_commanded_torque())
            .finish()
        } else {
            f.debug_tuple("M172TorqueAndTimerInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M172TorqueAndTimerInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_power_on_timer = u.float_in_range(0_f32..=12884800_f32)?;
        let inv_torque_feedback = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_commanded_torque = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        M172TorqueAndTimerInfo::new(inv_power_on_timer,inv_torque_feedback,inv_commanded_torque).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M194_Read_Write_Param_Response
///
/// - ID: 194 (0xc2)
/// - Size: 8 bytes
/// - Transmitter: INV
///
/// To write a parameter use message 0x0C1 with byte #2 set to 1 (write). To read a parameter use message 0x0C1 with byte #2 to set 0 (read).
#[derive(Clone, Copy)]
pub struct M194ReadWriteParamResponse {
    raw: [u8; 8],
}

impl M194ReadWriteParamResponse {
    pub const MESSAGE_ID: u32 = 194;
    
    pub const INV_PARAMETER_RESPONSE_DATA_MIN: i16 = -32768_i16;
    pub const INV_PARAMETER_RESPONSE_DATA_MAX: i16 = 32767_i16;
    pub const INV_PARAMETER_RESPONSE_ADDR_MIN: u16 = 0_u16;
    pub const INV_PARAMETER_RESPONSE_ADDR_MAX: u16 = 65535_u16;
    
    /// Construct new M194_Read_Write_Param_Response from values
    pub fn new(inv_parameter_response_data: i16, inv_parameter_response_write_ok: bool, inv_parameter_response_addr: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_parameter_response_data(inv_parameter_response_data)?;
        res.set_inv_parameter_response_write_ok(inv_parameter_response_write_ok)?;
        res.set_inv_parameter_response_addr(inv_parameter_response_addr)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Parameter_Response_Data
    ///
    /// Data from parameter message.  All data is 16 bits and is contained in bytes 4 and 5. Bytes 6 and 7 should be ignored.
    ///
    /// - Min: -32768
    /// - Max: 32767
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_parameter_response_data(&self) -> i16 {
        self.inv_parameter_response_data_raw()
    }
    
    /// Get raw value of INV_Parameter_Response_Data
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_parameter_response_data_raw(&self) -> i16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of INV_Parameter_Response_Data
    #[inline(always)]
    pub fn set_inv_parameter_response_data(&mut self, value: i16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -32768_i16 || 32767_i16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 194 });
        }
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Parameter_Response_Write_OK
    ///
    /// 0=Write failure, 1=Success
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_parameter_response_write_ok(&self) -> bool {
        self.inv_parameter_response_write_ok_raw()
    }
    
    /// Get raw value of INV_Parameter_Response_Write_OK
    ///
    /// - Start bit: 16
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_parameter_response_write_ok_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[16..17].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Parameter_Response_Write_OK
    #[inline(always)]
    pub fn set_inv_parameter_response_write_ok(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[16..17].store_le(value);
        Ok(())
    }
    
    /// INV_Parameter_Response_Addr
    ///
    /// Address of parameter response message data.
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_parameter_response_addr(&self) -> u16 {
        self.inv_parameter_response_addr_raw()
    }
    
    /// Get raw value of INV_Parameter_Response_Addr
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_parameter_response_addr_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_Parameter_Response_Addr
    #[inline(always)]
    pub fn set_inv_parameter_response_addr(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 194 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M194ReadWriteParamResponse {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M194ReadWriteParamResponse {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M194ReadWriteParamResponse")
                .field("inv_parameter_response_data", &self.inv_parameter_response_data())
                .field("inv_parameter_response_write_ok", &self.inv_parameter_response_write_ok())
                .field("inv_parameter_response_addr", &self.inv_parameter_response_addr())
            .finish()
        } else {
            f.debug_tuple("M194ReadWriteParamResponse").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M194ReadWriteParamResponse {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_parameter_response_data = u.int_in_range(-32768..=32767)?;
        let inv_parameter_response_write_ok = u.int_in_range(0..=1)? == 1;
        let inv_parameter_response_addr = u.int_in_range(0..=65535)?;
        M194ReadWriteParamResponse::new(inv_parameter_response_data,inv_parameter_response_write_ok,inv_parameter_response_addr).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M193_Read_Write_Param_Command
///
/// - ID: 193 (0xc1)
/// - Size: 8 bytes
///
/// To write a parameter use message 0x0C1 with byte #2 set to 1 (write). To read a parameter use message 0x0C1 with byte #2 to set 0 (read).
#[derive(Clone, Copy)]
pub struct M193ReadWriteParamCommand {
    raw: [u8; 8],
}

impl M193ReadWriteParamCommand {
    pub const MESSAGE_ID: u32 = 193;
    
    pub const VCU_INV_PARAMETER_DATA_MIN: i16 = -32768_i16;
    pub const VCU_INV_PARAMETER_DATA_MAX: i16 = 32767_i16;
    pub const VCU_INV_PARAMETER_ADDRESS_MIN: u16 = 0_u16;
    pub const VCU_INV_PARAMETER_ADDRESS_MAX: u16 = 65535_u16;
    
    /// Construct new M193_Read_Write_Param_Command from values
    pub fn new(vcu_inv_parameter_data: i16, vcu_inv_parameter_rw_command: bool, vcu_inv_parameter_address: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_inv_parameter_data(vcu_inv_parameter_data)?;
        res.set_vcu_inv_parameter_rw_command(vcu_inv_parameter_rw_command)?;
        res.set_vcu_inv_parameter_address(vcu_inv_parameter_address)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// VCU_INV_Parameter_Data
    ///
    /// Data to be written.  All data is 16 bits and is contained in bytes 4 and 5.
    ///
    /// - Min: -32768
    /// - Max: 32767
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_parameter_data(&self) -> i16 {
        self.vcu_inv_parameter_data_raw()
    }
    
    /// Get raw value of VCU_INV_Parameter_Data
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_inv_parameter_data_raw(&self) -> i16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of VCU_INV_Parameter_Data
    #[inline(always)]
    pub fn set_vcu_inv_parameter_data(&mut self, value: i16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -32768_i16 || 32767_i16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 193 });
        }
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Parameter_RW_Command
    ///
    /// 0=Read, 1=Write
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_parameter_rw_command(&self) -> bool {
        self.vcu_inv_parameter_rw_command_raw()
    }
    
    /// Get raw value of VCU_INV_Parameter_RW_Command
    ///
    /// - Start bit: 16
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_inv_parameter_rw_command_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[16..17].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_INV_Parameter_RW_Command
    #[inline(always)]
    pub fn set_vcu_inv_parameter_rw_command(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[16..17].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Parameter_Address
    ///
    /// Address of parameter to be written or read.
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_parameter_address(&self) -> u16 {
        self.vcu_inv_parameter_address_raw()
    }
    
    /// Get raw value of VCU_INV_Parameter_Address
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_inv_parameter_address_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of VCU_INV_Parameter_Address
    #[inline(always)]
    pub fn set_vcu_inv_parameter_address(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 193 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M193ReadWriteParamCommand {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M193ReadWriteParamCommand {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M193ReadWriteParamCommand")
                .field("vcu_inv_parameter_data", &self.vcu_inv_parameter_data())
                .field("vcu_inv_parameter_rw_command", &self.vcu_inv_parameter_rw_command())
                .field("vcu_inv_parameter_address", &self.vcu_inv_parameter_address())
            .finish()
        } else {
            f.debug_tuple("M193ReadWriteParamCommand").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M193ReadWriteParamCommand {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_inv_parameter_data = u.int_in_range(-32768..=32767)?;
        let vcu_inv_parameter_rw_command = u.int_in_range(0..=1)? == 1;
        let vcu_inv_parameter_address = u.int_in_range(0..=65535)?;
        M193ReadWriteParamCommand::new(vcu_inv_parameter_data,vcu_inv_parameter_rw_command,vcu_inv_parameter_address).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M192_Command_Message
///
/// - ID: 192 (0xc0)
/// - Size: 8 bytes
///
/// The command message is used to transmit data to the controller. This message is sent from a user supplied external controller to the PMxxx controller.
#[derive(Clone, Copy)]
pub struct M192CommandMessage {
    raw: [u8; 8],
}

impl M192CommandMessage {
    pub const MESSAGE_ID: u32 = 192;
    
    pub const VCU_INV_TORQUE_LIMIT_COMMAND_MIN: f32 = -3276.8_f32;
    pub const VCU_INV_TORQUE_LIMIT_COMMAND_MAX: f32 = 3276.7_f32;
    pub const VCU_INV_ROLLING_COUNTER_MIN: u8 = 0_u8;
    pub const VCU_INV_ROLLING_COUNTER_MAX: u8 = 15_u8;
    pub const VCU_INV_SPEED_COMMAND_MIN: i16 = -32768_i16;
    pub const VCU_INV_SPEED_COMMAND_MAX: i16 = 32767_i16;
    pub const VCU_INV_TORQUE_COMMAND_MIN: f32 = -3276.8_f32;
    pub const VCU_INV_TORQUE_COMMAND_MAX: f32 = 3276.7_f32;
    
    /// Construct new M192_Command_Message from values
    pub fn new(vcu_inv_torque_limit_command: f32, vcu_inv_rolling_counter: u8, vcu_inv_speed_mode_enable: bool, vcu_inv_inverter_discharge: bool, vcu_inv_inverter_enable: bool, vcu_inv_direction_command: bool, vcu_inv_speed_command: i16, vcu_inv_torque_command: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_vcu_inv_torque_limit_command(vcu_inv_torque_limit_command)?;
        res.set_vcu_inv_rolling_counter(vcu_inv_rolling_counter)?;
        res.set_vcu_inv_speed_mode_enable(vcu_inv_speed_mode_enable)?;
        res.set_vcu_inv_inverter_discharge(vcu_inv_inverter_discharge)?;
        res.set_vcu_inv_inverter_enable(vcu_inv_inverter_enable)?;
        res.set_vcu_inv_direction_command(vcu_inv_direction_command)?;
        res.set_vcu_inv_speed_command(vcu_inv_speed_command)?;
        res.set_vcu_inv_torque_command(vcu_inv_torque_command)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// VCU_INV_Torque_Limit_Command
    ///
    /// Torque Limit, set to 0 to keep default
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_torque_limit_command(&self) -> f32 {
        self.vcu_inv_torque_limit_command_raw()
    }
    
    /// Get raw value of VCU_INV_Torque_Limit_Command
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_inv_torque_limit_command_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of VCU_INV_Torque_Limit_Command
    #[inline(always)]
    pub fn set_vcu_inv_torque_limit_command(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 192 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Rolling_Counter
    ///
    /// Rolling Counter sent to inverter.  If used increment count with each message sent.  Otherwise can be set to 0.
    ///
    /// - Min: 0
    /// - Max: 15
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_rolling_counter(&self) -> u8 {
        self.vcu_inv_rolling_counter_raw()
    }
    
    /// Get raw value of VCU_INV_Rolling_Counter
    ///
    /// - Start bit: 44
    /// - Signal size: 4 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_inv_rolling_counter_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[44..48].load_le::<u8>();
        
        signal
    }
    
    /// Set value of VCU_INV_Rolling_Counter
    #[inline(always)]
    pub fn set_vcu_inv_rolling_counter(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 15_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 192 });
        }
        self.raw.view_bits_mut::<Lsb0>()[44..48].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Speed_Mode_Enable
    ///
    /// 0 = No change to mode, 1 = change to speed mode from torque mode
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_speed_mode_enable(&self) -> bool {
        self.vcu_inv_speed_mode_enable_raw()
    }
    
    /// Get raw value of VCU_INV_Speed_Mode_Enable
    ///
    /// - Start bit: 42
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_inv_speed_mode_enable_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[42..43].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_INV_Speed_Mode_Enable
    #[inline(always)]
    pub fn set_vcu_inv_speed_mode_enable(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[42..43].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Inverter_Discharge
    ///
    /// 0=Discharge Disable,1=Discharge Enable
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_inverter_discharge(&self) -> M192CommandMessageVcuInvInverterDischarge {
        let signal = self.raw.view_bits::<Lsb0>()[41..42].load_le::<u8>();
        
        match signal {
            0 => M192CommandMessageVcuInvInverterDischarge::DischargeDisable,
            1 => M192CommandMessageVcuInvInverterDischarge::DischargeEnableIfEepromParameterIsSet,
            _ => M192CommandMessageVcuInvInverterDischarge::_Other(self.vcu_inv_inverter_discharge_raw()),
        }
    }
    
    /// Get raw value of VCU_INV_Inverter_Discharge
    ///
    /// - Start bit: 41
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_inv_inverter_discharge_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[41..42].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_INV_Inverter_Discharge
    #[inline(always)]
    pub fn set_vcu_inv_inverter_discharge(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[41..42].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Inverter_Enable
    ///
    /// 0=Inverter OFF, 1 = Inverter ON
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_inverter_enable(&self) -> M192CommandMessageVcuInvInverterEnable {
        let signal = self.raw.view_bits::<Lsb0>()[40..41].load_le::<u8>();
        
        match signal {
            0 => M192CommandMessageVcuInvInverterEnable::TurnTheInverterOff,
            1 => M192CommandMessageVcuInvInverterEnable::TurnTheInverterOn,
            _ => M192CommandMessageVcuInvInverterEnable::_Other(self.vcu_inv_inverter_enable_raw()),
        }
    }
    
    /// Get raw value of VCU_INV_Inverter_Enable
    ///
    /// - Start bit: 40
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_inv_inverter_enable_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[40..41].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_INV_Inverter_Enable
    #[inline(always)]
    pub fn set_vcu_inv_inverter_enable(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[40..41].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Direction_Command
    ///
    /// 0=Reverse, 1=Forward.  Forward is positive motor speed.
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_direction_command(&self) -> M192CommandMessageVcuInvDirectionCommand {
        let signal = self.raw.view_bits::<Lsb0>()[32..33].load_le::<u8>();
        
        match signal {
            0 => M192CommandMessageVcuInvDirectionCommand::Cw,
            1 => M192CommandMessageVcuInvDirectionCommand::Ccw,
            _ => M192CommandMessageVcuInvDirectionCommand::_Other(self.vcu_inv_direction_command_raw()),
        }
    }
    
    /// Get raw value of VCU_INV_Direction_Command
    ///
    /// - Start bit: 32
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn vcu_inv_direction_command_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[32..33].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of VCU_INV_Direction_Command
    #[inline(always)]
    pub fn set_vcu_inv_direction_command(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[32..33].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Speed_Command
    ///
    /// Speed command used when in speed mode
    ///
    /// - Min: -32768
    /// - Max: 32767
    /// - Unit: "angular_speed:rpm"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_speed_command(&self) -> i16 {
        self.vcu_inv_speed_command_raw()
    }
    
    /// Get raw value of VCU_INV_Speed_Command
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_inv_speed_command_raw(&self) -> i16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of VCU_INV_Speed_Command
    #[inline(always)]
    pub fn set_vcu_inv_speed_command(&mut self, value: i16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -32768_i16 || 32767_i16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 192 });
        }
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// VCU_INV_Torque_Command
    ///
    /// Torque command when in torque mode
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn vcu_inv_torque_command(&self) -> f32 {
        self.vcu_inv_torque_command_raw()
    }
    
    /// Get raw value of VCU_INV_Torque_Command
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn vcu_inv_torque_command_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of VCU_INV_Torque_Command
    #[inline(always)]
    pub fn set_vcu_inv_torque_command(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 192 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M192CommandMessage {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M192CommandMessage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M192CommandMessage")
                .field("vcu_inv_torque_limit_command", &self.vcu_inv_torque_limit_command())
                .field("vcu_inv_rolling_counter", &self.vcu_inv_rolling_counter())
                .field("vcu_inv_speed_mode_enable", &self.vcu_inv_speed_mode_enable())
                .field("vcu_inv_inverter_discharge", &self.vcu_inv_inverter_discharge())
                .field("vcu_inv_inverter_enable", &self.vcu_inv_inverter_enable())
                .field("vcu_inv_direction_command", &self.vcu_inv_direction_command())
                .field("vcu_inv_speed_command", &self.vcu_inv_speed_command())
                .field("vcu_inv_torque_command", &self.vcu_inv_torque_command())
            .finish()
        } else {
            f.debug_tuple("M192CommandMessage").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M192CommandMessage {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let vcu_inv_torque_limit_command = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let vcu_inv_rolling_counter = u.int_in_range(0..=15)?;
        let vcu_inv_speed_mode_enable = u.int_in_range(0..=1)? == 1;
        let vcu_inv_inverter_discharge = u.int_in_range(0..=1)? == 1;
        let vcu_inv_inverter_enable = u.int_in_range(0..=1)? == 1;
        let vcu_inv_direction_command = u.int_in_range(0..=1)? == 1;
        let vcu_inv_speed_command = u.int_in_range(-32768..=32767)?;
        let vcu_inv_torque_command = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        M192CommandMessage::new(vcu_inv_torque_limit_command,vcu_inv_rolling_counter,vcu_inv_speed_mode_enable,vcu_inv_inverter_discharge,vcu_inv_inverter_enable,vcu_inv_direction_command,vcu_inv_speed_command,vcu_inv_torque_command).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for VCU_INV_Inverter_Discharge
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M192CommandMessageVcuInvInverterDischarge {
    DischargeDisable,
    DischargeEnableIfEepromParameterIsSet,
    _Other(bool),
}

impl From<M192CommandMessageVcuInvInverterDischarge> for bool {
    fn from(val: M192CommandMessageVcuInvInverterDischarge) -> bool {
        match val {
            M192CommandMessageVcuInvInverterDischarge::DischargeDisable => false,
            M192CommandMessageVcuInvInverterDischarge::DischargeEnableIfEepromParameterIsSet => true,
            M192CommandMessageVcuInvInverterDischarge::_Other(x) => x,
        }
    }
}

/// Defined values for VCU_INV_Inverter_Enable
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M192CommandMessageVcuInvInverterEnable {
    TurnTheInverterOff,
    TurnTheInverterOn,
    _Other(bool),
}

impl From<M192CommandMessageVcuInvInverterEnable> for bool {
    fn from(val: M192CommandMessageVcuInvInverterEnable) -> bool {
        match val {
            M192CommandMessageVcuInvInverterEnable::TurnTheInverterOff => false,
            M192CommandMessageVcuInvInverterEnable::TurnTheInverterOn => true,
            M192CommandMessageVcuInvInverterEnable::_Other(x) => x,
        }
    }
}

/// Defined values for VCU_INV_Direction_Command
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M192CommandMessageVcuInvDirectionCommand {
    Cw,
    Ccw,
    _Other(bool),
}

impl From<M192CommandMessageVcuInvDirectionCommand> for bool {
    fn from(val: M192CommandMessageVcuInvDirectionCommand) -> bool {
        match val {
            M192CommandMessageVcuInvDirectionCommand::Cw => false,
            M192CommandMessageVcuInvDirectionCommand::Ccw => true,
            M192CommandMessageVcuInvDirectionCommand::_Other(x) => x,
        }
    }
}


/// M171_Fault_Codes
///
/// - ID: 171 (0xab)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M171FaultCodes {
    raw: [u8; 8],
}

impl M171FaultCodes {
    pub const MESSAGE_ID: u32 = 171;
    
    pub const INV_RUN_FAULT_HI_MIN: u16 = 0_u16;
    pub const INV_RUN_FAULT_HI_MAX: u16 = 65535_u16;
    pub const INV_RUN_FAULT_LO_MIN: u16 = 0_u16;
    pub const INV_RUN_FAULT_LO_MAX: u16 = 65535_u16;
    pub const INV_POST_FAULT_HI_MIN: u16 = 0_u16;
    pub const INV_POST_FAULT_HI_MAX: u16 = 65535_u16;
    pub const INV_POST_FAULT_LO_MIN: u16 = 0_u16;
    pub const INV_POST_FAULT_LO_MAX: u16 = 65535_u16;
    
    /// Construct new M171_Fault_Codes from values
    pub fn new(inv_run_fault_hi: u16, inv_run_fault_lo: u16, inv_post_fault_hi: u16, inv_post_fault_lo: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_run_fault_hi(inv_run_fault_hi)?;
        res.set_inv_run_fault_lo(inv_run_fault_lo)?;
        res.set_inv_post_fault_hi(inv_post_fault_hi)?;
        res.set_inv_post_fault_lo(inv_post_fault_lo)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Run_Fault_Hi
    ///
    /// Each bit represents a fault. Please refer to PM100 Users Manual for details.
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_run_fault_hi(&self) -> u16 {
        self.inv_run_fault_hi_raw()
    }
    
    /// Get raw value of INV_Run_Fault_Hi
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_run_fault_hi_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_Run_Fault_Hi
    #[inline(always)]
    pub fn set_inv_run_fault_hi(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 171 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Run_Fault_Lo
    ///
    /// Each bit represents a fault. Please refer to PM100 Users Manual for details.
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_run_fault_lo(&self) -> u16 {
        self.inv_run_fault_lo_raw()
    }
    
    /// Get raw value of INV_Run_Fault_Lo
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_run_fault_lo_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_Run_Fault_Lo
    #[inline(always)]
    pub fn set_inv_run_fault_lo(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 171 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Post_Fault_Hi
    ///
    /// Each bit represents a fault. Please refer to PM100 Users Manual for details.
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_post_fault_hi(&self) -> u16 {
        self.inv_post_fault_hi_raw()
    }
    
    /// Get raw value of INV_Post_Fault_Hi
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_post_fault_hi_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_Post_Fault_Hi
    #[inline(always)]
    pub fn set_inv_post_fault_hi(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 171 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Post_Fault_Lo
    ///
    /// Each bit represents a fault. Please refer to PM100 Users Manual for details.
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_post_fault_lo(&self) -> u16 {
        self.inv_post_fault_lo_raw()
    }
    
    /// Get raw value of INV_Post_Fault_Lo
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_post_fault_lo_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_Post_Fault_Lo
    #[inline(always)]
    pub fn set_inv_post_fault_lo(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 171 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M171FaultCodes {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M171FaultCodes {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M171FaultCodes")
                .field("inv_run_fault_hi", &self.inv_run_fault_hi())
                .field("inv_run_fault_lo", &self.inv_run_fault_lo())
                .field("inv_post_fault_hi", &self.inv_post_fault_hi())
                .field("inv_post_fault_lo", &self.inv_post_fault_lo())
            .finish()
        } else {
            f.debug_tuple("M171FaultCodes").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M171FaultCodes {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_run_fault_hi = u.int_in_range(0..=65535)?;
        let inv_run_fault_lo = u.int_in_range(0..=65535)?;
        let inv_post_fault_hi = u.int_in_range(0..=65535)?;
        let inv_post_fault_lo = u.int_in_range(0..=65535)?;
        M171FaultCodes::new(inv_run_fault_hi,inv_run_fault_lo,inv_post_fault_hi,inv_post_fault_lo).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M170_Internal_States
///
/// - ID: 170 (0xaa)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M170InternalStates {
    raw: [u8; 8],
}

impl M170InternalStates {
    pub const MESSAGE_ID: u32 = 170;
    
    pub const INV_ROLLING_COUNTER_MIN: u8 = 0_u8;
    pub const INV_ROLLING_COUNTER_MAX: u8 = 15_u8;
    pub const INV_INVERTER_DISCHARGE_STATE_MIN: u8 = 0_u8;
    pub const INV_INVERTER_DISCHARGE_STATE_MAX: u8 = 7_u8;
    pub const INV_INVERTER_STATE_MIN: u8 = 0_u8;
    pub const INV_INVERTER_STATE_MAX: u8 = 255_u8;
    pub const INV_PWM_FREQUENCY_MIN: u8 = 0_u8;
    pub const INV_PWM_FREQUENCY_MAX: u8 = 255_u8;
    pub const INV_VSM_STATE_MIN: u8 = 0_u8;
    pub const INV_VSM_STATE_MAX: u8 = 15_u8;
    
    /// Construct new M170_Internal_States from values
    pub fn new(inv_limit_stall_burst_model: bool, inv_limit_coolant_derating: bool, inv_low_speed_limiting: bool, inv_limit_hot_spot: bool, inv_limit_max_speed: bool, inv_bms_limiting_motor_torque: bool, inv_bms_active: bool, inv_direction_command: bool, inv_inverter_enable_lockout: bool, inv_key_switch_start_status: bool, inv_bms_limiting_regen_torque: bool, inv_burst_model_mode: bool, inv_inverter_enable_state: bool, inv_rolling_counter: u8, inv_inverter_command_mode: bool, inv_inverter_discharge_state: u8, inv_self_sensing_assist_enable: bool, inv_inverter_run_mode: bool, inv_relay_6_status: bool, inv_relay_5_status: bool, inv_relay_4_status: bool, inv_relay_3_status: bool, inv_relay_2_status: bool, inv_relay_1_status: bool, inv_inverter_state: u8, inv_pwm_frequency: u8, inv_vsm_state: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_limit_stall_burst_model(inv_limit_stall_burst_model)?;
        res.set_inv_limit_coolant_derating(inv_limit_coolant_derating)?;
        res.set_inv_low_speed_limiting(inv_low_speed_limiting)?;
        res.set_inv_limit_hot_spot(inv_limit_hot_spot)?;
        res.set_inv_limit_max_speed(inv_limit_max_speed)?;
        res.set_inv_bms_limiting_motor_torque(inv_bms_limiting_motor_torque)?;
        res.set_inv_bms_active(inv_bms_active)?;
        res.set_inv_direction_command(inv_direction_command)?;
        res.set_inv_inverter_enable_lockout(inv_inverter_enable_lockout)?;
        res.set_inv_key_switch_start_status(inv_key_switch_start_status)?;
        res.set_inv_bms_limiting_regen_torque(inv_bms_limiting_regen_torque)?;
        res.set_inv_burst_model_mode(inv_burst_model_mode)?;
        res.set_inv_inverter_enable_state(inv_inverter_enable_state)?;
        res.set_inv_rolling_counter(inv_rolling_counter)?;
        res.set_inv_inverter_command_mode(inv_inverter_command_mode)?;
        res.set_inv_inverter_discharge_state(inv_inverter_discharge_state)?;
        res.set_inv_self_sensing_assist_enable(inv_self_sensing_assist_enable)?;
        res.set_inv_inverter_run_mode(inv_inverter_run_mode)?;
        res.set_inv_relay_6_status(inv_relay_6_status)?;
        res.set_inv_relay_5_status(inv_relay_5_status)?;
        res.set_inv_relay_4_status(inv_relay_4_status)?;
        res.set_inv_relay_3_status(inv_relay_3_status)?;
        res.set_inv_relay_2_status(inv_relay_2_status)?;
        res.set_inv_relay_1_status(inv_relay_1_status)?;
        res.set_inv_inverter_state(inv_inverter_state)?;
        res.set_inv_pwm_frequency(inv_pwm_frequency)?;
        res.set_inv_vsm_state(inv_vsm_state)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Limit_Stall_Burst_Model
    ///
    /// 0 = Not limiting, 1 = Limiting
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_limit_stall_burst_model(&self) -> M170InternalStatesInvLimitStallBurstModel {
        let signal = self.raw.view_bits::<Lsb0>()[63..64].load_le::<u8>();
        
        match signal {
            0 => M170InternalStatesInvLimitStallBurstModel::NotLimiting,
            1 => M170InternalStatesInvLimitStallBurstModel::Limiting,
            _ => M170InternalStatesInvLimitStallBurstModel::_Other(self.inv_limit_stall_burst_model_raw()),
        }
    }
    
    /// Get raw value of INV_Limit_Stall_Burst_Model
    ///
    /// - Start bit: 63
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_limit_stall_burst_model_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[63..64].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Limit_Stall_Burst_Model
    #[inline(always)]
    pub fn set_inv_limit_stall_burst_model(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[63..64].store_le(value);
        Ok(())
    }
    
    /// INV_Limit_Coolant_Derating
    ///
    /// 0 = Not limiting, 1 = Limiting
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_limit_coolant_derating(&self) -> bool {
        self.inv_limit_coolant_derating_raw()
    }
    
    /// Get raw value of INV_Limit_Coolant_Derating
    ///
    /// - Start bit: 62
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_limit_coolant_derating_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[62..63].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Limit_Coolant_Derating
    #[inline(always)]
    pub fn set_inv_limit_coolant_derating(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[62..63].store_le(value);
        Ok(())
    }
    
    /// INV_Low_Speed_Limiting
    ///
    /// Indicates that motor current is being limited due to low motor electrical frequency.
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_low_speed_limiting(&self) -> bool {
        self.inv_low_speed_limiting_raw()
    }
    
    /// Get raw value of INV_Low_Speed_Limiting
    ///
    /// - Start bit: 61
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_low_speed_limiting_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[61..62].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Low_Speed_Limiting
    #[inline(always)]
    pub fn set_inv_low_speed_limiting(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[61..62].store_le(value);
        Ok(())
    }
    
    /// INV_Limit_Hot_Spot
    ///
    /// Indicates that torque is being limited to limit the hot spot temp
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_limit_hot_spot(&self) -> bool {
        self.inv_limit_hot_spot_raw()
    }
    
    /// Get raw value of INV_Limit_Hot_Spot
    ///
    /// - Start bit: 60
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_limit_hot_spot_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[60..61].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Limit_Hot_Spot
    #[inline(always)]
    pub fn set_inv_limit_hot_spot(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[60..61].store_le(value);
        Ok(())
    }
    
    /// INV_Limit_Max_Speed
    ///
    /// Indicates that torque is being modified to limit the speed.
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_limit_max_speed(&self) -> bool {
        self.inv_limit_max_speed_raw()
    }
    
    /// Get raw value of INV_Limit_Max_Speed
    ///
    /// - Start bit: 59
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_limit_max_speed_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[59..60].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Limit_Max_Speed
    #[inline(always)]
    pub fn set_inv_limit_max_speed(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[59..60].store_le(value);
        Ok(())
    }
    
    /// INV_BMS_Limiting_Motor_Torque
    ///
    /// 0 = Not Limiting, 1 = Limiting
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_bms_limiting_motor_torque(&self) -> bool {
        self.inv_bms_limiting_motor_torque_raw()
    }
    
    /// Get raw value of INV_BMS_Limiting_Motor_Torque
    ///
    /// - Start bit: 58
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_bms_limiting_motor_torque_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[58..59].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_BMS_Limiting_Motor_Torque
    #[inline(always)]
    pub fn set_inv_bms_limiting_motor_torque(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[58..59].store_le(value);
        Ok(())
    }
    
    /// INV_BMS_Active
    ///
    /// 0 = BMS Not Active, 1 = BMS Active
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_bms_active(&self) -> bool {
        self.inv_bms_active_raw()
    }
    
    /// Get raw value of INV_BMS_Active
    ///
    /// - Start bit: 57
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_bms_active_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[57..58].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_BMS_Active
    #[inline(always)]
    pub fn set_inv_bms_active(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[57..58].store_le(value);
        Ok(())
    }
    
    /// INV_Direction_Command
    ///
    /// 1 = Forward 0 = 'Reverse' if inverter enabled  & 'Stopped' if inverter is disabled
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_direction_command(&self) -> bool {
        self.inv_direction_command_raw()
    }
    
    /// Get raw value of INV_Direction_Command
    ///
    /// - Start bit: 56
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_direction_command_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[56..57].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Direction_Command
    #[inline(always)]
    pub fn set_inv_direction_command(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[56..57].store_le(value);
        Ok(())
    }
    
    /// INV_Inverter_Enable_Lockout
    ///
    /// 0=Lockout Disabled, 1=Lockout Enabled
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_inverter_enable_lockout(&self) -> bool {
        self.inv_inverter_enable_lockout_raw()
    }
    
    /// Get raw value of INV_Inverter_Enable_Lockout
    ///
    /// - Start bit: 55
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_inverter_enable_lockout_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[55..56].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Inverter_Enable_Lockout
    #[inline(always)]
    pub fn set_inv_inverter_enable_lockout(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[55..56].store_le(value);
        Ok(())
    }
    
    /// INV_Key_Switch_Start_Status
    ///
    /// 0 = OFF, 1 = ON
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_key_switch_start_status(&self) -> bool {
        self.inv_key_switch_start_status_raw()
    }
    
    /// Get raw value of INV_Key_Switch_Start_Status
    ///
    /// - Start bit: 54
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_key_switch_start_status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[54..55].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Key_Switch_Start_Status
    #[inline(always)]
    pub fn set_inv_key_switch_start_status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[54..55].store_le(value);
        Ok(())
    }
    
    /// INV_BMS_Limiting_Regen_Torque
    ///
    /// 0 = Not Limiting, 1 = Limiting
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_bms_limiting_regen_torque(&self) -> bool {
        self.inv_bms_limiting_regen_torque_raw()
    }
    
    /// Get raw value of INV_BMS_Limiting_Regen_Torque
    ///
    /// - Start bit: 50
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_bms_limiting_regen_torque_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[50..51].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_BMS_Limiting_Regen_Torque
    #[inline(always)]
    pub fn set_inv_bms_limiting_regen_torque(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[50..51].store_le(value);
        Ok(())
    }
    
    /// INV_Burst_Model_Mode
    ///
    /// 0 = Stall, 1 = High Speed
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_burst_model_mode(&self) -> M170InternalStatesInvBurstModelMode {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        match signal {
            0 => M170InternalStatesInvBurstModelMode::Stall,
            1 => M170InternalStatesInvBurstModelMode::HighSpeed,
            _ => M170InternalStatesInvBurstModelMode::_Other(self.inv_burst_model_mode_raw()),
        }
    }
    
    /// Get raw value of INV_Burst_Model_Mode
    ///
    /// - Start bit: 49
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_burst_model_mode_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[49..50].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Burst_Model_Mode
    #[inline(always)]
    pub fn set_inv_burst_model_mode(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[49..50].store_le(value);
        Ok(())
    }
    
    /// INV_Inverter_Enable_State
    ///
    /// 0=Inverter Disabled, 1=Inverter Enabled
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_inverter_enable_state(&self) -> bool {
        self.inv_inverter_enable_state_raw()
    }
    
    /// Get raw value of INV_Inverter_Enable_State
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_inverter_enable_state_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Inverter_Enable_State
    #[inline(always)]
    pub fn set_inv_inverter_enable_state(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// INV_Rolling_Counter
    ///
    /// Rolling Counter value
    ///
    /// - Min: 0
    /// - Max: 15
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_rolling_counter(&self) -> u8 {
        self.inv_rolling_counter_raw()
    }
    
    /// Get raw value of INV_Rolling_Counter
    ///
    /// - Start bit: 44
    /// - Signal size: 4 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_rolling_counter_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[44..48].load_le::<u8>();
        
        signal
    }
    
    /// Set value of INV_Rolling_Counter
    #[inline(always)]
    pub fn set_inv_rolling_counter(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 15_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 170 });
        }
        self.raw.view_bits_mut::<Lsb0>()[44..48].store_le(value);
        Ok(())
    }
    
    /// INV_Inverter_Command_Mode
    ///
    /// 0=CAN mode, 1=VSM mode
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_inverter_command_mode(&self) -> bool {
        self.inv_inverter_command_mode_raw()
    }
    
    /// Get raw value of INV_Inverter_Command_Mode
    ///
    /// - Start bit: 40
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_inverter_command_mode_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[40..41].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Inverter_Command_Mode
    #[inline(always)]
    pub fn set_inv_inverter_command_mode(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[40..41].store_le(value);
        Ok(())
    }
    
    /// INV_Inverter_Discharge_State
    ///
    /// 0 = Disabled, 1 = Enabled, 2 = Speed Check, 3 = Active, 4 = Complete, 5 = Error, 6 = Override, 7 = Timeout
    ///
    /// - Min: 0
    /// - Max: 7
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_inverter_discharge_state(&self) -> M170InternalStatesInvInverterDischargeState {
        let signal = self.raw.view_bits::<Lsb0>()[37..40].load_le::<u8>();
        
        match signal {
            0 => M170InternalStatesInvInverterDischargeState::Disabled,
            1 => M170InternalStatesInvInverterDischargeState::Enabled,
            2 => M170InternalStatesInvInverterDischargeState::SpeedCheck,
            3 => M170InternalStatesInvInverterDischargeState::Active,
            4 => M170InternalStatesInvInverterDischargeState::Complete,
            5 => M170InternalStatesInvInverterDischargeState::Error,
            6 => M170InternalStatesInvInverterDischargeState::XOverride,
            7 => M170InternalStatesInvInverterDischargeState::Timeout,
            _ => M170InternalStatesInvInverterDischargeState::_Other(self.inv_inverter_discharge_state_raw()),
        }
    }
    
    /// Get raw value of INV_Inverter_Discharge_State
    ///
    /// - Start bit: 37
    /// - Signal size: 3 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_inverter_discharge_state_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[37..40].load_le::<u8>();
        
        signal
    }
    
    /// Set value of INV_Inverter_Discharge_State
    #[inline(always)]
    pub fn set_inv_inverter_discharge_state(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 7_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 170 });
        }
        self.raw.view_bits_mut::<Lsb0>()[37..40].store_le(value);
        Ok(())
    }
    
    /// INV_Self_Sensing_Assist_Enable
    ///
    /// 1 = Self Sensing Assist Enabled, 0 = Self Sensing Assist Disabled
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_self_sensing_assist_enable(&self) -> bool {
        self.inv_self_sensing_assist_enable_raw()
    }
    
    /// Get raw value of INV_Self_Sensing_Assist_Enable
    ///
    /// - Start bit: 33
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_self_sensing_assist_enable_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[33..34].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Self_Sensing_Assist_Enable
    #[inline(always)]
    pub fn set_inv_self_sensing_assist_enable(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[33..34].store_le(value);
        Ok(())
    }
    
    /// INV_Inverter_Run_Mode
    ///
    /// 0=Torque Mode, 1=Speed Mode
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_inverter_run_mode(&self) -> bool {
        self.inv_inverter_run_mode_raw()
    }
    
    /// Get raw value of INV_Inverter_Run_Mode
    ///
    /// - Start bit: 32
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_inverter_run_mode_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[32..33].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Inverter_Run_Mode
    #[inline(always)]
    pub fn set_inv_inverter_run_mode(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[32..33].store_le(value);
        Ok(())
    }
    
    /// INV_Relay_6_Status
    ///
    /// 0=OFF, 1=ON
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_relay_6_status(&self) -> bool {
        self.inv_relay_6_status_raw()
    }
    
    /// Get raw value of INV_Relay_6_Status
    ///
    /// - Start bit: 29
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_relay_6_status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[29..30].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Relay_6_Status
    #[inline(always)]
    pub fn set_inv_relay_6_status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[29..30].store_le(value);
        Ok(())
    }
    
    /// INV_Relay_5_Status
    ///
    /// 0=OFF, 1=ON
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_relay_5_status(&self) -> bool {
        self.inv_relay_5_status_raw()
    }
    
    /// Get raw value of INV_Relay_5_Status
    ///
    /// - Start bit: 28
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_relay_5_status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[28..29].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Relay_5_Status
    #[inline(always)]
    pub fn set_inv_relay_5_status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[28..29].store_le(value);
        Ok(())
    }
    
    /// INV_Relay_4_Status
    ///
    /// 0=OFF, 1=ON
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_relay_4_status(&self) -> bool {
        self.inv_relay_4_status_raw()
    }
    
    /// Get raw value of INV_Relay_4_Status
    ///
    /// - Start bit: 27
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_relay_4_status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[27..28].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Relay_4_Status
    #[inline(always)]
    pub fn set_inv_relay_4_status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[27..28].store_le(value);
        Ok(())
    }
    
    /// INV_Relay_3_Status
    ///
    /// 0=OFF, 1=ON
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_relay_3_status(&self) -> bool {
        self.inv_relay_3_status_raw()
    }
    
    /// Get raw value of INV_Relay_3_Status
    ///
    /// - Start bit: 26
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_relay_3_status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[26..27].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Relay_3_Status
    #[inline(always)]
    pub fn set_inv_relay_3_status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[26..27].store_le(value);
        Ok(())
    }
    
    /// INV_Relay_2_Status
    ///
    /// 0=OFF, 1=ON
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_relay_2_status(&self) -> bool {
        self.inv_relay_2_status_raw()
    }
    
    /// Get raw value of INV_Relay_2_Status
    ///
    /// - Start bit: 25
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_relay_2_status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[25..26].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Relay_2_Status
    #[inline(always)]
    pub fn set_inv_relay_2_status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[25..26].store_le(value);
        Ok(())
    }
    
    /// INV_Relay_1_Status
    ///
    /// 0=OFF, 1=ON
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_relay_1_status(&self) -> bool {
        self.inv_relay_1_status_raw()
    }
    
    /// Get raw value of INV_Relay_1_Status
    ///
    /// - Start bit: 24
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_relay_1_status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[24..25].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Relay_1_Status
    #[inline(always)]
    pub fn set_inv_relay_1_status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[24..25].store_le(value);
        Ok(())
    }
    
    /// INV_Inverter_State
    ///
    /// Different states for the inverter state machine
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_inverter_state(&self) -> M170InternalStatesInvInverterState {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        match signal {
            0 => M170InternalStatesInvInverterState::PowerUp,
            1 => M170InternalStatesInvInverterState::Stop,
            2 => M170InternalStatesInvInverterState::OpenLoop,
            3 => M170InternalStatesInvInverterState::ClosedLoop,
            4 => M170InternalStatesInvInverterState::InternalState,
            5 => M170InternalStatesInvInverterState::InternalState,
            6 => M170InternalStatesInvInverterState::InternalState,
            7 => M170InternalStatesInvInverterState::InternalState,
            8 => M170InternalStatesInvInverterState::IdleRun,
            9 => M170InternalStatesInvInverterState::IdleStop,
            10 => M170InternalStatesInvInverterState::InternalState,
            11 => M170InternalStatesInvInverterState::InternState,
            12 => M170InternalStatesInvInverterState::InternalState,
            _ => M170InternalStatesInvInverterState::_Other(self.inv_inverter_state_raw()),
        }
    }
    
    /// Get raw value of INV_Inverter_State
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_inverter_state_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        signal
    }
    
    /// Set value of INV_Inverter_State
    #[inline(always)]
    pub fn set_inv_inverter_state(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 170 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// INV_PWM_Frequency
    ///
    /// The current active PWM frequency
    ///
    /// - Min: 0
    /// - Max: 255
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_pwm_frequency(&self) -> u8 {
        self.inv_pwm_frequency_raw()
    }
    
    /// Get raw value of INV_PWM_Frequency
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_pwm_frequency_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        signal
    }
    
    /// Set value of INV_PWM_Frequency
    #[inline(always)]
    pub fn set_inv_pwm_frequency(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 255_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 170 });
        }
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// INV_VSM_State
    ///
    /// Different states for the vehicle state machine
    ///
    /// - Min: 0
    /// - Max: 15
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_vsm_state(&self) -> M170InternalStatesInvVsmState {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        match signal {
            0 => M170InternalStatesInvVsmState::VsmStartState,
            1 => M170InternalStatesInvVsmState::PreChargeInitState,
            2 => M170InternalStatesInvVsmState::PreChargeActiveState,
            3 => M170InternalStatesInvVsmState::PreChargeCompleteState,
            4 => M170InternalStatesInvVsmState::VsmWaitState,
            5 => M170InternalStatesInvVsmState::VsmReadyState,
            6 => M170InternalStatesInvVsmState::MotorRunningState,
            7 => M170InternalStatesInvVsmState::BlinkFaultCodeState,
            14 => M170InternalStatesInvVsmState::ShutdownStateForKeySwitchMode1,
            15 => M170InternalStatesInvVsmState::ResetTheInverter,
            _ => M170InternalStatesInvVsmState::_Other(self.inv_vsm_state_raw()),
        }
    }
    
    /// Get raw value of INV_VSM_State
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_vsm_state_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of INV_VSM_State
    #[inline(always)]
    pub fn set_inv_vsm_state(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 15_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 170 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M170InternalStates {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M170InternalStates {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M170InternalStates")
                .field("inv_limit_stall_burst_model", &self.inv_limit_stall_burst_model())
                .field("inv_limit_coolant_derating", &self.inv_limit_coolant_derating())
                .field("inv_low_speed_limiting", &self.inv_low_speed_limiting())
                .field("inv_limit_hot_spot", &self.inv_limit_hot_spot())
                .field("inv_limit_max_speed", &self.inv_limit_max_speed())
                .field("inv_bms_limiting_motor_torque", &self.inv_bms_limiting_motor_torque())
                .field("inv_bms_active", &self.inv_bms_active())
                .field("inv_direction_command", &self.inv_direction_command())
                .field("inv_inverter_enable_lockout", &self.inv_inverter_enable_lockout())
                .field("inv_key_switch_start_status", &self.inv_key_switch_start_status())
                .field("inv_bms_limiting_regen_torque", &self.inv_bms_limiting_regen_torque())
                .field("inv_burst_model_mode", &self.inv_burst_model_mode())
                .field("inv_inverter_enable_state", &self.inv_inverter_enable_state())
                .field("inv_rolling_counter", &self.inv_rolling_counter())
                .field("inv_inverter_command_mode", &self.inv_inverter_command_mode())
                .field("inv_inverter_discharge_state", &self.inv_inverter_discharge_state())
                .field("inv_self_sensing_assist_enable", &self.inv_self_sensing_assist_enable())
                .field("inv_inverter_run_mode", &self.inv_inverter_run_mode())
                .field("inv_relay_6_status", &self.inv_relay_6_status())
                .field("inv_relay_5_status", &self.inv_relay_5_status())
                .field("inv_relay_4_status", &self.inv_relay_4_status())
                .field("inv_relay_3_status", &self.inv_relay_3_status())
                .field("inv_relay_2_status", &self.inv_relay_2_status())
                .field("inv_relay_1_status", &self.inv_relay_1_status())
                .field("inv_inverter_state", &self.inv_inverter_state())
                .field("inv_pwm_frequency", &self.inv_pwm_frequency())
                .field("inv_vsm_state", &self.inv_vsm_state())
            .finish()
        } else {
            f.debug_tuple("M170InternalStates").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M170InternalStates {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_limit_stall_burst_model = u.int_in_range(0..=1)? == 1;
        let inv_limit_coolant_derating = u.int_in_range(0..=1)? == 1;
        let inv_low_speed_limiting = u.int_in_range(0..=1)? == 1;
        let inv_limit_hot_spot = u.int_in_range(0..=1)? == 1;
        let inv_limit_max_speed = u.int_in_range(0..=1)? == 1;
        let inv_bms_limiting_motor_torque = u.int_in_range(0..=1)? == 1;
        let inv_bms_active = u.int_in_range(0..=1)? == 1;
        let inv_direction_command = u.int_in_range(0..=1)? == 1;
        let inv_inverter_enable_lockout = u.int_in_range(0..=1)? == 1;
        let inv_key_switch_start_status = u.int_in_range(0..=1)? == 1;
        let inv_bms_limiting_regen_torque = u.int_in_range(0..=1)? == 1;
        let inv_burst_model_mode = u.int_in_range(0..=1)? == 1;
        let inv_inverter_enable_state = u.int_in_range(0..=1)? == 1;
        let inv_rolling_counter = u.int_in_range(0..=15)?;
        let inv_inverter_command_mode = u.int_in_range(0..=1)? == 1;
        let inv_inverter_discharge_state = u.int_in_range(0..=7)?;
        let inv_self_sensing_assist_enable = u.int_in_range(0..=1)? == 1;
        let inv_inverter_run_mode = u.int_in_range(0..=1)? == 1;
        let inv_relay_6_status = u.int_in_range(0..=1)? == 1;
        let inv_relay_5_status = u.int_in_range(0..=1)? == 1;
        let inv_relay_4_status = u.int_in_range(0..=1)? == 1;
        let inv_relay_3_status = u.int_in_range(0..=1)? == 1;
        let inv_relay_2_status = u.int_in_range(0..=1)? == 1;
        let inv_relay_1_status = u.int_in_range(0..=1)? == 1;
        let inv_inverter_state = u.int_in_range(0..=255)?;
        let inv_pwm_frequency = u.int_in_range(0..=255)?;
        let inv_vsm_state = u.int_in_range(0..=15)?;
        M170InternalStates::new(inv_limit_stall_burst_model,inv_limit_coolant_derating,inv_low_speed_limiting,inv_limit_hot_spot,inv_limit_max_speed,inv_bms_limiting_motor_torque,inv_bms_active,inv_direction_command,inv_inverter_enable_lockout,inv_key_switch_start_status,inv_bms_limiting_regen_torque,inv_burst_model_mode,inv_inverter_enable_state,inv_rolling_counter,inv_inverter_command_mode,inv_inverter_discharge_state,inv_self_sensing_assist_enable,inv_inverter_run_mode,inv_relay_6_status,inv_relay_5_status,inv_relay_4_status,inv_relay_3_status,inv_relay_2_status,inv_relay_1_status,inv_inverter_state,inv_pwm_frequency,inv_vsm_state).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for INV_Limit_Stall_Burst_Model
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M170InternalStatesInvLimitStallBurstModel {
    NotLimiting,
    Limiting,
    _Other(bool),
}

impl From<M170InternalStatesInvLimitStallBurstModel> for bool {
    fn from(val: M170InternalStatesInvLimitStallBurstModel) -> bool {
        match val {
            M170InternalStatesInvLimitStallBurstModel::NotLimiting => false,
            M170InternalStatesInvLimitStallBurstModel::Limiting => true,
            M170InternalStatesInvLimitStallBurstModel::_Other(x) => x,
        }
    }
}

/// Defined values for INV_Burst_Model_Mode
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M170InternalStatesInvBurstModelMode {
    Stall,
    HighSpeed,
    _Other(bool),
}

impl From<M170InternalStatesInvBurstModelMode> for bool {
    fn from(val: M170InternalStatesInvBurstModelMode) -> bool {
        match val {
            M170InternalStatesInvBurstModelMode::Stall => false,
            M170InternalStatesInvBurstModelMode::HighSpeed => true,
            M170InternalStatesInvBurstModelMode::_Other(x) => x,
        }
    }
}

/// Defined values for INV_Inverter_Discharge_State
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M170InternalStatesInvInverterDischargeState {
    Disabled,
    Enabled,
    SpeedCheck,
    Active,
    Complete,
    Error,
    XOverride,
    Timeout,
    _Other(u8),
}

impl From<M170InternalStatesInvInverterDischargeState> for u8 {
    fn from(val: M170InternalStatesInvInverterDischargeState) -> u8 {
        match val {
            M170InternalStatesInvInverterDischargeState::Disabled => 0,
            M170InternalStatesInvInverterDischargeState::Enabled => 1,
            M170InternalStatesInvInverterDischargeState::SpeedCheck => 2,
            M170InternalStatesInvInverterDischargeState::Active => 3,
            M170InternalStatesInvInverterDischargeState::Complete => 4,
            M170InternalStatesInvInverterDischargeState::Error => 5,
            M170InternalStatesInvInverterDischargeState::XOverride => 6,
            M170InternalStatesInvInverterDischargeState::Timeout => 7,
            M170InternalStatesInvInverterDischargeState::_Other(x) => x,
        }
    }
}

/// Defined values for INV_Inverter_State
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M170InternalStatesInvInverterState {
    PowerUp,
    Stop,
    OpenLoop,
    ClosedLoop,
    InternalState,
    InternalState,
    InternalState,
    InternalState,
    IdleRun,
    IdleStop,
    InternalState,
    InternState,
    InternalState,
    _Other(u8),
}

impl From<M170InternalStatesInvInverterState> for u8 {
    fn from(val: M170InternalStatesInvInverterState) -> u8 {
        match val {
            M170InternalStatesInvInverterState::PowerUp => 0,
            M170InternalStatesInvInverterState::Stop => 1,
            M170InternalStatesInvInverterState::OpenLoop => 2,
            M170InternalStatesInvInverterState::ClosedLoop => 3,
            M170InternalStatesInvInverterState::InternalState => 4,
            M170InternalStatesInvInverterState::InternalState => 5,
            M170InternalStatesInvInverterState::InternalState => 6,
            M170InternalStatesInvInverterState::InternalState => 7,
            M170InternalStatesInvInverterState::IdleRun => 8,
            M170InternalStatesInvInverterState::IdleStop => 9,
            M170InternalStatesInvInverterState::InternalState => 10,
            M170InternalStatesInvInverterState::InternState => 11,
            M170InternalStatesInvInverterState::InternalState => 12,
            M170InternalStatesInvInverterState::_Other(x) => x,
        }
    }
}

/// Defined values for INV_VSM_State
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M170InternalStatesInvVsmState {
    VsmStartState,
    PreChargeInitState,
    PreChargeActiveState,
    PreChargeCompleteState,
    VsmWaitState,
    VsmReadyState,
    MotorRunningState,
    BlinkFaultCodeState,
    ShutdownStateForKeySwitchMode1,
    ResetTheInverter,
    _Other(u8),
}

impl From<M170InternalStatesInvVsmState> for u8 {
    fn from(val: M170InternalStatesInvVsmState) -> u8 {
        match val {
            M170InternalStatesInvVsmState::VsmStartState => 0,
            M170InternalStatesInvVsmState::PreChargeInitState => 1,
            M170InternalStatesInvVsmState::PreChargeActiveState => 2,
            M170InternalStatesInvVsmState::PreChargeCompleteState => 3,
            M170InternalStatesInvVsmState::VsmWaitState => 4,
            M170InternalStatesInvVsmState::VsmReadyState => 5,
            M170InternalStatesInvVsmState::MotorRunningState => 6,
            M170InternalStatesInvVsmState::BlinkFaultCodeState => 7,
            M170InternalStatesInvVsmState::ShutdownStateForKeySwitchMode1 => 14,
            M170InternalStatesInvVsmState::ResetTheInverter => 15,
            M170InternalStatesInvVsmState::_Other(x) => x,
        }
    }
}


/// M169_Internal_Voltages
///
/// - ID: 169 (0xa9)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M169InternalVoltages {
    raw: [u8; 8],
}

impl M169InternalVoltages {
    pub const MESSAGE_ID: u32 = 169;
    
    pub const INV_REF_VOLTAGE_12_0_MIN: f32 = -327.68_f32;
    pub const INV_REF_VOLTAGE_12_0_MAX: f32 = 327.67_f32;
    pub const INV_REF_VOLTAGE_5_0_MIN: f32 = -327.68_f32;
    pub const INV_REF_VOLTAGE_5_0_MAX: f32 = 327.67_f32;
    pub const INV_REF_VOLTAGE_2_5_MIN: f32 = -327.68_f32;
    pub const INV_REF_VOLTAGE_2_5_MAX: f32 = 327.67_f32;
    pub const INV_REF_VOLTAGE_1_5_MIN: f32 = -327.68_f32;
    pub const INV_REF_VOLTAGE_1_5_MAX: f32 = 327.67_f32;
    
    /// Construct new M169_Internal_Voltages from values
    pub fn new(inv_ref_voltage_12_0: f32, inv_ref_voltage_5_0: f32, inv_ref_voltage_2_5: f32, inv_ref_voltage_1_5: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_ref_voltage_12_0(inv_ref_voltage_12_0)?;
        res.set_inv_ref_voltage_5_0(inv_ref_voltage_5_0)?;
        res.set_inv_ref_voltage_2_5(inv_ref_voltage_2_5)?;
        res.set_inv_ref_voltage_1_5(inv_ref_voltage_1_5)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Ref_Voltage_12_0
    ///
    /// 12V Input Voltage
    ///
    /// - Min: -327.68
    /// - Max: 327.67
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_ref_voltage_12_0(&self) -> f32 {
        self.inv_ref_voltage_12_0_raw()
    }
    
    /// Get raw value of INV_Ref_Voltage_12_0
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_ref_voltage_12_0_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Ref_Voltage_12_0
    #[inline(always)]
    pub fn set_inv_ref_voltage_12_0(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -327.68_f32 || 327.67_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 169 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Ref_Voltage_5_0
    ///
    /// Transducer voltage
    ///
    /// - Min: -327.68
    /// - Max: 327.67
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_ref_voltage_5_0(&self) -> f32 {
        self.inv_ref_voltage_5_0_raw()
    }
    
    /// Get raw value of INV_Ref_Voltage_5_0
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_ref_voltage_5_0_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Ref_Voltage_5_0
    #[inline(always)]
    pub fn set_inv_ref_voltage_5_0(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -327.68_f32 || 327.67_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 169 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Ref_Voltage_2_5
    ///
    /// Internal reference voltage
    ///
    /// - Min: -327.68
    /// - Max: 327.67
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_ref_voltage_2_5(&self) -> f32 {
        self.inv_ref_voltage_2_5_raw()
    }
    
    /// Get raw value of INV_Ref_Voltage_2_5
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_ref_voltage_2_5_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Ref_Voltage_2_5
    #[inline(always)]
    pub fn set_inv_ref_voltage_2_5(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -327.68_f32 || 327.67_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 169 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Ref_Voltage_1_5
    ///
    /// Internal reference voltage
    ///
    /// - Min: -327.68
    /// - Max: 327.67
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_ref_voltage_1_5(&self) -> f32 {
        self.inv_ref_voltage_1_5_raw()
    }
    
    /// Get raw value of INV_Ref_Voltage_1_5
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_ref_voltage_1_5_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Ref_Voltage_1_5
    #[inline(always)]
    pub fn set_inv_ref_voltage_1_5(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -327.68_f32 || 327.67_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 169 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M169InternalVoltages {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M169InternalVoltages {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M169InternalVoltages")
                .field("inv_ref_voltage_12_0", &self.inv_ref_voltage_12_0())
                .field("inv_ref_voltage_5_0", &self.inv_ref_voltage_5_0())
                .field("inv_ref_voltage_2_5", &self.inv_ref_voltage_2_5())
                .field("inv_ref_voltage_1_5", &self.inv_ref_voltage_1_5())
            .finish()
        } else {
            f.debug_tuple("M169InternalVoltages").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M169InternalVoltages {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_ref_voltage_12_0 = u.float_in_range(-327.68_f32..=327.67_f32)?;
        let inv_ref_voltage_5_0 = u.float_in_range(-327.68_f32..=327.67_f32)?;
        let inv_ref_voltage_2_5 = u.float_in_range(-327.68_f32..=327.67_f32)?;
        let inv_ref_voltage_1_5 = u.float_in_range(-327.68_f32..=327.67_f32)?;
        M169InternalVoltages::new(inv_ref_voltage_12_0,inv_ref_voltage_5_0,inv_ref_voltage_2_5,inv_ref_voltage_1_5).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M168_Flux_ID_IQ_Info
///
/// - ID: 168 (0xa8)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M168FluxIdIqInfo {
    raw: [u8; 8],
}

impl M168FluxIdIqInfo {
    pub const MESSAGE_ID: u32 = 168;
    
    pub const INV_IQ_MIN: f32 = -3276.8_f32;
    pub const INV_IQ_MAX: f32 = 3276.7_f32;
    pub const INV_ID_MIN: f32 = -3276.8_f32;
    pub const INV_ID_MAX: f32 = 3276.7_f32;
    pub const INV_VQ_FF_MIN: f32 = -3276.7_f32;
    pub const INV_VQ_FF_MAX: f32 = 3276.7_f32;
    pub const INV_VD_FF_MIN: f32 = -3276.7_f32;
    pub const INV_VD_FF_MAX: f32 = 3276.7_f32;
    
    /// Construct new M168_Flux_ID_IQ_Info from values
    pub fn new(inv_iq: f32, inv_id: f32, inv_vq_ff: f32, inv_vd_ff: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_iq(inv_iq)?;
        res.set_inv_id(inv_id)?;
        res.set_inv_vq_ff(inv_vq_ff)?;
        res.set_inv_vd_ff(inv_vd_ff)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Iq
    ///
    /// The measured Iq current
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_iq(&self) -> f32 {
        self.inv_iq_raw()
    }
    
    /// Get raw value of INV_Iq
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_iq_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Iq
    #[inline(always)]
    pub fn set_inv_iq(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 168 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Id
    ///
    /// The measured Id current
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_id(&self) -> f32 {
        self.inv_id_raw()
    }
    
    /// Get raw value of INV_Id
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_id_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Id
    #[inline(always)]
    pub fn set_inv_id(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 168 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Vq_ff
    ///
    /// The q-axis voltage feedforward
    ///
    /// - Min: -3276.7
    /// - Max: 3276.7
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_vq_ff(&self) -> f32 {
        self.inv_vq_ff_raw()
    }
    
    /// Get raw value of INV_Vq_ff
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_vq_ff_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Vq_ff
    #[inline(always)]
    pub fn set_inv_vq_ff(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.7_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 168 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Vd_ff
    ///
    /// The d-axis voltage feedforward
    ///
    /// - Min: -3276.7
    /// - Max: 3276.7
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_vd_ff(&self) -> f32 {
        self.inv_vd_ff_raw()
    }
    
    /// Get raw value of INV_Vd_ff
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_vd_ff_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Vd_ff
    #[inline(always)]
    pub fn set_inv_vd_ff(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.7_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 168 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M168FluxIdIqInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M168FluxIdIqInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M168FluxIdIqInfo")
                .field("inv_iq", &self.inv_iq())
                .field("inv_id", &self.inv_id())
                .field("inv_vq_ff", &self.inv_vq_ff())
                .field("inv_vd_ff", &self.inv_vd_ff())
            .finish()
        } else {
            f.debug_tuple("M168FluxIdIqInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M168FluxIdIqInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_iq = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_id = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_vq_ff = u.float_in_range(-3276.7_f32..=3276.7_f32)?;
        let inv_vd_ff = u.float_in_range(-3276.7_f32..=3276.7_f32)?;
        M168FluxIdIqInfo::new(inv_iq,inv_id,inv_vq_ff,inv_vd_ff).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M167_Voltage_Info
///
/// - ID: 167 (0xa7)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M167VoltageInfo {
    raw: [u8; 8],
}

impl M167VoltageInfo {
    pub const MESSAGE_ID: u32 = 167;
    
    pub const INV_VBC_VQ_VOLTAGE_MIN: f32 = -3276.8_f32;
    pub const INV_VBC_VQ_VOLTAGE_MAX: f32 = 3276.7_f32;
    pub const INV_VAB_VD_VOLTAGE_MIN: f32 = -3276.8_f32;
    pub const INV_VAB_VD_VOLTAGE_MAX: f32 = 3276.7_f32;
    pub const INV_OUTPUT_VOLTAGE_MIN: f32 = -3276.8_f32;
    pub const INV_OUTPUT_VOLTAGE_MAX: f32 = 3276.7_f32;
    pub const INV_DC_BUS_VOLTAGE_MIN: f32 = -3276.8_f32;
    pub const INV_DC_BUS_VOLTAGE_MAX: f32 = 3276.7_f32;
    
    /// Construct new M167_Voltage_Info from values
    pub fn new(inv_vbc_vq_voltage: f32, inv_vab_vd_voltage: f32, inv_output_voltage: f32, inv_dc_bus_voltage: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_vbc_vq_voltage(inv_vbc_vq_voltage)?;
        res.set_inv_vab_vd_voltage(inv_vab_vd_voltage)?;
        res.set_inv_output_voltage(inv_output_voltage)?;
        res.set_inv_dc_bus_voltage(inv_dc_bus_voltage)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_VBC_Vq_Voltage
    ///
    /// Measured value of the voltage between Phase B and Phase C
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_vbc_vq_voltage(&self) -> f32 {
        self.inv_vbc_vq_voltage_raw()
    }
    
    /// Get raw value of INV_VBC_Vq_Voltage
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_vbc_vq_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_VBC_Vq_Voltage
    #[inline(always)]
    pub fn set_inv_vbc_vq_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 167 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_VAB_Vd_Voltage
    ///
    /// Measured value of the voltage betwen phase A and Phase B
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_vab_vd_voltage(&self) -> f32 {
        self.inv_vab_vd_voltage_raw()
    }
    
    /// Get raw value of INV_VAB_Vd_Voltage
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_vab_vd_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_VAB_Vd_Voltage
    #[inline(always)]
    pub fn set_inv_vab_vd_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 167 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Output_Voltage
    ///
    /// The calculated value of the output voltage, in peak line-neutral volts
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_output_voltage(&self) -> f32 {
        self.inv_output_voltage_raw()
    }
    
    /// Get raw value of INV_Output_Voltage
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_output_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Output_Voltage
    #[inline(always)]
    pub fn set_inv_output_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 167 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_DC_Bus_Voltage
    ///
    /// The actual measured value of the DC bus voltage
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_dc_bus_voltage(&self) -> f32 {
        self.inv_dc_bus_voltage_raw()
    }
    
    /// Get raw value of INV_DC_Bus_Voltage
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_dc_bus_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_DC_Bus_Voltage
    #[inline(always)]
    pub fn set_inv_dc_bus_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 167 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M167VoltageInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M167VoltageInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M167VoltageInfo")
                .field("inv_vbc_vq_voltage", &self.inv_vbc_vq_voltage())
                .field("inv_vab_vd_voltage", &self.inv_vab_vd_voltage())
                .field("inv_output_voltage", &self.inv_output_voltage())
                .field("inv_dc_bus_voltage", &self.inv_dc_bus_voltage())
            .finish()
        } else {
            f.debug_tuple("M167VoltageInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M167VoltageInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_vbc_vq_voltage = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_vab_vd_voltage = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_output_voltage = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_dc_bus_voltage = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        M167VoltageInfo::new(inv_vbc_vq_voltage,inv_vab_vd_voltage,inv_output_voltage,inv_dc_bus_voltage).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M166_Current_Info
///
/// - ID: 166 (0xa6)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M166CurrentInfo {
    raw: [u8; 8],
}

impl M166CurrentInfo {
    pub const MESSAGE_ID: u32 = 166;
    
    pub const INV_DC_BUS_CURRENT_MIN: f32 = -3276.8_f32;
    pub const INV_DC_BUS_CURRENT_MAX: f32 = 3276.7_f32;
    pub const INV_PHASE_C_CURRENT_MIN: f32 = -3276.8_f32;
    pub const INV_PHASE_C_CURRENT_MAX: f32 = 3276.7_f32;
    pub const INV_PHASE_B_CURRENT_MIN: f32 = -3276.8_f32;
    pub const INV_PHASE_B_CURRENT_MAX: f32 = 3276.7_f32;
    pub const INV_PHASE_A_CURRENT_MIN: f32 = -3276.8_f32;
    pub const INV_PHASE_A_CURRENT_MAX: f32 = 3276.7_f32;
    
    /// Construct new M166_Current_Info from values
    pub fn new(inv_dc_bus_current: f32, inv_phase_c_current: f32, inv_phase_b_current: f32, inv_phase_a_current: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_dc_bus_current(inv_dc_bus_current)?;
        res.set_inv_phase_c_current(inv_phase_c_current)?;
        res.set_inv_phase_b_current(inv_phase_b_current)?;
        res.set_inv_phase_a_current(inv_phase_a_current)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_DC_Bus_Current
    ///
    /// The Calculated DC Bus Current
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_dc_bus_current(&self) -> f32 {
        self.inv_dc_bus_current_raw()
    }
    
    /// Get raw value of INV_DC_Bus_Current
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_dc_bus_current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_DC_Bus_Current
    #[inline(always)]
    pub fn set_inv_dc_bus_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 166 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Phase_C_Current
    ///
    /// The measured value of Phase C current
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_phase_c_current(&self) -> f32 {
        self.inv_phase_c_current_raw()
    }
    
    /// Get raw value of INV_Phase_C_Current
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_phase_c_current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Phase_C_Current
    #[inline(always)]
    pub fn set_inv_phase_c_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 166 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Phase_B_Current
    ///
    /// The measured value of Phase B current
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_phase_b_current(&self) -> f32 {
        self.inv_phase_b_current_raw()
    }
    
    /// Get raw value of INV_Phase_B_Current
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_phase_b_current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Phase_B_Current
    #[inline(always)]
    pub fn set_inv_phase_b_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 166 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Phase_A_Current
    ///
    /// The measured value of Phase A current
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_phase_a_current(&self) -> f32 {
        self.inv_phase_a_current_raw()
    }
    
    /// Get raw value of INV_Phase_A_Current
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_phase_a_current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Phase_A_Current
    #[inline(always)]
    pub fn set_inv_phase_a_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 166 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M166CurrentInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M166CurrentInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M166CurrentInfo")
                .field("inv_dc_bus_current", &self.inv_dc_bus_current())
                .field("inv_phase_c_current", &self.inv_phase_c_current())
                .field("inv_phase_b_current", &self.inv_phase_b_current())
                .field("inv_phase_a_current", &self.inv_phase_a_current())
            .finish()
        } else {
            f.debug_tuple("M166CurrentInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M166CurrentInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_dc_bus_current = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_phase_c_current = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_phase_b_current = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_phase_a_current = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        M166CurrentInfo::new(inv_dc_bus_current,inv_phase_c_current,inv_phase_b_current,inv_phase_a_current).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M165_Motor_Position_Info
///
/// - ID: 165 (0xa5)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M165MotorPositionInfo {
    raw: [u8; 8],
}

impl M165MotorPositionInfo {
    pub const MESSAGE_ID: u32 = 165;
    
    pub const INV_DELTA_RESOLVER_FILTERED_MIN: f32 = -3276.8_f32;
    pub const INV_DELTA_RESOLVER_FILTERED_MAX: f32 = 3276.7_f32;
    pub const INV_ELECTRICAL_OUTPUT_FREQUENCY_MIN: f32 = -3276.8_f32;
    pub const INV_ELECTRICAL_OUTPUT_FREQUENCY_MAX: f32 = 3276.7_f32;
    pub const INV_MOTOR_SPEED_MIN: i16 = -32768_i16;
    pub const INV_MOTOR_SPEED_MAX: i16 = 32767_i16;
    pub const INV_MOTOR_ANGLE_ELECTRICAL_MIN: f32 = 0_f32;
    pub const INV_MOTOR_ANGLE_ELECTRICAL_MAX: f32 = 6553.5_f32;
    
    /// Construct new M165_Motor_Position_Info from values
    pub fn new(inv_delta_resolver_filtered: f32, inv_electrical_output_frequency: f32, inv_motor_speed: i16, inv_motor_angle_electrical: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_delta_resolver_filtered(inv_delta_resolver_filtered)?;
        res.set_inv_electrical_output_frequency(inv_electrical_output_frequency)?;
        res.set_inv_motor_speed(inv_motor_speed)?;
        res.set_inv_motor_angle_electrical(inv_motor_angle_electrical)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Delta_Resolver_Filtered
    ///
    /// Used in calibration of resolver angle adjustment.
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "angle:deg"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_delta_resolver_filtered(&self) -> f32 {
        self.inv_delta_resolver_filtered_raw()
    }
    
    /// Get raw value of INV_Delta_Resolver_Filtered
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_delta_resolver_filtered_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Delta_Resolver_Filtered
    #[inline(always)]
    pub fn set_inv_delta_resolver_filtered(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 165 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Electrical_Output_Frequency
    ///
    /// The actual electrical frequency of the inverter
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "frequency:Hz"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_electrical_output_frequency(&self) -> f32 {
        self.inv_electrical_output_frequency_raw()
    }
    
    /// Get raw value of INV_Electrical_Output_Frequency
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_electrical_output_frequency_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Electrical_Output_Frequency
    #[inline(always)]
    pub fn set_inv_electrical_output_frequency(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 165 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Motor_Speed
    ///
    /// The measured speed of the motor
    ///
    /// - Min: -32768
    /// - Max: 32767
    /// - Unit: "angular_speed:rpm"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_motor_speed(&self) -> i16 {
        self.inv_motor_speed_raw()
    }
    
    /// Get raw value of INV_Motor_Speed
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_motor_speed_raw(&self) -> i16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of INV_Motor_Speed
    #[inline(always)]
    pub fn set_inv_motor_speed(&mut self, value: i16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -32768_i16 || 32767_i16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 165 });
        }
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Motor_Angle_Electrical
    ///
    /// The Electrical Angle of the motor as read by the encoder or resolver
    ///
    /// - Min: 0
    /// - Max: 6553.5
    /// - Unit: "angle:deg"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_motor_angle_electrical(&self) -> f32 {
        self.inv_motor_angle_electrical_raw()
    }
    
    /// Get raw value of INV_Motor_Angle_Electrical
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_motor_angle_electrical_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Motor_Angle_Electrical
    #[inline(always)]
    pub fn set_inv_motor_angle_electrical(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 6553.5_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 165 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M165MotorPositionInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M165MotorPositionInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M165MotorPositionInfo")
                .field("inv_delta_resolver_filtered", &self.inv_delta_resolver_filtered())
                .field("inv_electrical_output_frequency", &self.inv_electrical_output_frequency())
                .field("inv_motor_speed", &self.inv_motor_speed())
                .field("inv_motor_angle_electrical", &self.inv_motor_angle_electrical())
            .finish()
        } else {
            f.debug_tuple("M165MotorPositionInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M165MotorPositionInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_delta_resolver_filtered = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_electrical_output_frequency = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_motor_speed = u.int_in_range(-32768..=32767)?;
        let inv_motor_angle_electrical = u.float_in_range(0_f32..=6553.5_f32)?;
        M165MotorPositionInfo::new(inv_delta_resolver_filtered,inv_electrical_output_frequency,inv_motor_speed,inv_motor_angle_electrical).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M164_Digital_Input_Status
///
/// - ID: 164 (0xa4)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M164DigitalInputStatus {
    raw: [u8; 8],
}

impl M164DigitalInputStatus {
    pub const MESSAGE_ID: u32 = 164;
    
    
    /// Construct new M164_Digital_Input_Status from values
    pub fn new(inv_digital_input_8: bool, inv_digital_input_7: bool, inv_digital_input_6: bool, inv_digital_input_5: bool, inv_digital_input_4: bool, inv_digital_input_3: bool, inv_digital_input_2: bool, inv_digital_input_1: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_digital_input_8(inv_digital_input_8)?;
        res.set_inv_digital_input_7(inv_digital_input_7)?;
        res.set_inv_digital_input_6(inv_digital_input_6)?;
        res.set_inv_digital_input_5(inv_digital_input_5)?;
        res.set_inv_digital_input_4(inv_digital_input_4)?;
        res.set_inv_digital_input_3(inv_digital_input_3)?;
        res.set_inv_digital_input_2(inv_digital_input_2)?;
        res.set_inv_digital_input_1(inv_digital_input_1)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Digital_Input_8
    ///
    /// Status of Digital Input #8
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_digital_input_8(&self) -> bool {
        self.inv_digital_input_8_raw()
    }
    
    /// Get raw value of INV_Digital_Input_8
    ///
    /// - Start bit: 56
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_digital_input_8_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[56..57].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Digital_Input_8
    #[inline(always)]
    pub fn set_inv_digital_input_8(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[56..57].store_le(value);
        Ok(())
    }
    
    /// INV_Digital_Input_7
    ///
    /// Status of Digital Input #7
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_digital_input_7(&self) -> bool {
        self.inv_digital_input_7_raw()
    }
    
    /// Get raw value of INV_Digital_Input_7
    ///
    /// - Start bit: 48
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_digital_input_7_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[48..49].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Digital_Input_7
    #[inline(always)]
    pub fn set_inv_digital_input_7(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[48..49].store_le(value);
        Ok(())
    }
    
    /// INV_Digital_Input_6
    ///
    /// Status of Digital Input #6
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_digital_input_6(&self) -> bool {
        self.inv_digital_input_6_raw()
    }
    
    /// Get raw value of INV_Digital_Input_6
    ///
    /// - Start bit: 40
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_digital_input_6_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[40..41].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Digital_Input_6
    #[inline(always)]
    pub fn set_inv_digital_input_6(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[40..41].store_le(value);
        Ok(())
    }
    
    /// INV_Digital_Input_5
    ///
    /// Status of Digital Input #5
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_digital_input_5(&self) -> bool {
        self.inv_digital_input_5_raw()
    }
    
    /// Get raw value of INV_Digital_Input_5
    ///
    /// - Start bit: 32
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_digital_input_5_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[32..33].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Digital_Input_5
    #[inline(always)]
    pub fn set_inv_digital_input_5(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[32..33].store_le(value);
        Ok(())
    }
    
    /// INV_Digital_Input_4
    ///
    /// Status of Digital Input #4
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_digital_input_4(&self) -> bool {
        self.inv_digital_input_4_raw()
    }
    
    /// Get raw value of INV_Digital_Input_4
    ///
    /// - Start bit: 24
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_digital_input_4_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[24..25].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Digital_Input_4
    #[inline(always)]
    pub fn set_inv_digital_input_4(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[24..25].store_le(value);
        Ok(())
    }
    
    /// INV_Digital_Input_3
    ///
    /// Status of Digital Input #3
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_digital_input_3(&self) -> bool {
        self.inv_digital_input_3_raw()
    }
    
    /// Get raw value of INV_Digital_Input_3
    ///
    /// - Start bit: 16
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_digital_input_3_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[16..17].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Digital_Input_3
    #[inline(always)]
    pub fn set_inv_digital_input_3(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[16..17].store_le(value);
        Ok(())
    }
    
    /// INV_Digital_Input_2
    ///
    /// Status of Digital Input #2
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_digital_input_2(&self) -> bool {
        self.inv_digital_input_2_raw()
    }
    
    /// Get raw value of INV_Digital_Input_2
    ///
    /// - Start bit: 8
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_digital_input_2_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[8..9].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Digital_Input_2
    #[inline(always)]
    pub fn set_inv_digital_input_2(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[8..9].store_le(value);
        Ok(())
    }
    
    /// INV_Digital_Input_1
    ///
    /// Status of Digital Input #1
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_digital_input_1(&self) -> bool {
        self.inv_digital_input_1_raw()
    }
    
    /// Get raw value of INV_Digital_Input_1
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_digital_input_1_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of INV_Digital_Input_1
    #[inline(always)]
    pub fn set_inv_digital_input_1(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M164DigitalInputStatus {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M164DigitalInputStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M164DigitalInputStatus")
                .field("inv_digital_input_8", &self.inv_digital_input_8())
                .field("inv_digital_input_7", &self.inv_digital_input_7())
                .field("inv_digital_input_6", &self.inv_digital_input_6())
                .field("inv_digital_input_5", &self.inv_digital_input_5())
                .field("inv_digital_input_4", &self.inv_digital_input_4())
                .field("inv_digital_input_3", &self.inv_digital_input_3())
                .field("inv_digital_input_2", &self.inv_digital_input_2())
                .field("inv_digital_input_1", &self.inv_digital_input_1())
            .finish()
        } else {
            f.debug_tuple("M164DigitalInputStatus").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M164DigitalInputStatus {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_digital_input_8 = u.int_in_range(0..=1)? == 1;
        let inv_digital_input_7 = u.int_in_range(0..=1)? == 1;
        let inv_digital_input_6 = u.int_in_range(0..=1)? == 1;
        let inv_digital_input_5 = u.int_in_range(0..=1)? == 1;
        let inv_digital_input_4 = u.int_in_range(0..=1)? == 1;
        let inv_digital_input_3 = u.int_in_range(0..=1)? == 1;
        let inv_digital_input_2 = u.int_in_range(0..=1)? == 1;
        let inv_digital_input_1 = u.int_in_range(0..=1)? == 1;
        M164DigitalInputStatus::new(inv_digital_input_8,inv_digital_input_7,inv_digital_input_6,inv_digital_input_5,inv_digital_input_4,inv_digital_input_3,inv_digital_input_2,inv_digital_input_1).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M163_Analog_Input_Voltages
///
/// - ID: 163 (0xa3)
/// - Size: 8 bytes
/// - Transmitter: INV
///
/// A mostly useless message.
#[derive(Clone, Copy)]
pub struct M163AnalogInputVoltages {
    raw: [u8; 8],
}

impl M163AnalogInputVoltages {
    pub const MESSAGE_ID: u32 = 163;
    
    pub const INV_ANALOG_INPUT_6_MIN: f32 = 0_f32;
    pub const INV_ANALOG_INPUT_6_MAX: f32 = 10.23_f32;
    pub const INV_ANALOG_INPUT_5_MIN: f32 = 0_f32;
    pub const INV_ANALOG_INPUT_5_MAX: f32 = 10.23_f32;
    pub const INV_ANALOG_INPUT_4_MIN: f32 = 0_f32;
    pub const INV_ANALOG_INPUT_4_MAX: f32 = 10.23_f32;
    pub const INV_ANALOG_INPUT_3_MIN: f32 = 0_f32;
    pub const INV_ANALOG_INPUT_3_MAX: f32 = 10.23_f32;
    pub const INV_ANALOG_INPUT_2_MIN: f32 = 0_f32;
    pub const INV_ANALOG_INPUT_2_MAX: f32 = 10.23_f32;
    pub const INV_ANALOG_INPUT_1_MIN: f32 = 0_f32;
    pub const INV_ANALOG_INPUT_1_MAX: f32 = 10.23_f32;
    
    /// Construct new M163_Analog_Input_Voltages from values
    pub fn new(inv_analog_input_6: f32, inv_analog_input_5: f32, inv_analog_input_4: f32, inv_analog_input_3: f32, inv_analog_input_2: f32, inv_analog_input_1: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_analog_input_6(inv_analog_input_6)?;
        res.set_inv_analog_input_5(inv_analog_input_5)?;
        res.set_inv_analog_input_4(inv_analog_input_4)?;
        res.set_inv_analog_input_3(inv_analog_input_3)?;
        res.set_inv_analog_input_2(inv_analog_input_2)?;
        res.set_inv_analog_input_1(inv_analog_input_1)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Analog_Input_6
    ///
    /// Voltage on Analog Input #6
    ///
    /// - Min: 0
    /// - Max: 10.23
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_analog_input_6(&self) -> f32 {
        self.inv_analog_input_6_raw()
    }
    
    /// Get raw value of INV_Analog_Input_6
    ///
    /// - Start bit: 52
    /// - Signal size: 10 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_analog_input_6_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[52..62].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Analog_Input_6
    #[inline(always)]
    pub fn set_inv_analog_input_6(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 10.23_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 163 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[52..62].store_le(value);
        Ok(())
    }
    
    /// INV_Analog_Input_5
    ///
    /// Voltage on Analog Input #5
    ///
    /// - Min: 0
    /// - Max: 10.23
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_analog_input_5(&self) -> f32 {
        self.inv_analog_input_5_raw()
    }
    
    /// Get raw value of INV_Analog_Input_5
    ///
    /// - Start bit: 42
    /// - Signal size: 10 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_analog_input_5_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[42..52].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Analog_Input_5
    #[inline(always)]
    pub fn set_inv_analog_input_5(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 10.23_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 163 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[42..52].store_le(value);
        Ok(())
    }
    
    /// INV_Analog_Input_4
    ///
    /// Voltage on Analog Input #4
    ///
    /// - Min: 0
    /// - Max: 10.23
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_analog_input_4(&self) -> f32 {
        self.inv_analog_input_4_raw()
    }
    
    /// Get raw value of INV_Analog_Input_4
    ///
    /// - Start bit: 32
    /// - Signal size: 10 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_analog_input_4_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..42].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Analog_Input_4
    #[inline(always)]
    pub fn set_inv_analog_input_4(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 10.23_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 163 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[32..42].store_le(value);
        Ok(())
    }
    
    /// INV_Analog_Input_3
    ///
    /// Voltage on Analog Input #3
    ///
    /// - Min: 0
    /// - Max: 10.23
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_analog_input_3(&self) -> f32 {
        self.inv_analog_input_3_raw()
    }
    
    /// Get raw value of INV_Analog_Input_3
    ///
    /// - Start bit: 20
    /// - Signal size: 10 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_analog_input_3_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[20..30].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Analog_Input_3
    #[inline(always)]
    pub fn set_inv_analog_input_3(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 10.23_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 163 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[20..30].store_le(value);
        Ok(())
    }
    
    /// INV_Analog_Input_2
    ///
    /// Voltage on Analog Input #2
    ///
    /// - Min: 0
    /// - Max: 10.23
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_analog_input_2(&self) -> f32 {
        self.inv_analog_input_2_raw()
    }
    
    /// Get raw value of INV_Analog_Input_2
    ///
    /// - Start bit: 10
    /// - Signal size: 10 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_analog_input_2_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[10..20].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Analog_Input_2
    #[inline(always)]
    pub fn set_inv_analog_input_2(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 10.23_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 163 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[10..20].store_le(value);
        Ok(())
    }
    
    /// INV_Analog_Input_1
    ///
    /// Voltage on Analog Input #1
    ///
    /// - Min: 0
    /// - Max: 10.23
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_analog_input_1(&self) -> f32 {
        self.inv_analog_input_1_raw()
    }
    
    /// Get raw value of INV_Analog_Input_1
    ///
    /// - Start bit: 0
    /// - Signal size: 10 bits
    /// - Factor: 0.01
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_analog_input_1_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..10].load_le::<u16>();
        
        let factor = 0.01_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Analog_Input_1
    #[inline(always)]
    pub fn set_inv_analog_input_1(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 10.23_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 163 });
        }
        let factor = 0.01_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..10].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M163AnalogInputVoltages {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M163AnalogInputVoltages {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M163AnalogInputVoltages")
                .field("inv_analog_input_6", &self.inv_analog_input_6())
                .field("inv_analog_input_5", &self.inv_analog_input_5())
                .field("inv_analog_input_4", &self.inv_analog_input_4())
                .field("inv_analog_input_3", &self.inv_analog_input_3())
                .field("inv_analog_input_2", &self.inv_analog_input_2())
                .field("inv_analog_input_1", &self.inv_analog_input_1())
            .finish()
        } else {
            f.debug_tuple("M163AnalogInputVoltages").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M163AnalogInputVoltages {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_analog_input_6 = u.float_in_range(0_f32..=10.23_f32)?;
        let inv_analog_input_5 = u.float_in_range(0_f32..=10.23_f32)?;
        let inv_analog_input_4 = u.float_in_range(0_f32..=10.23_f32)?;
        let inv_analog_input_3 = u.float_in_range(0_f32..=10.23_f32)?;
        let inv_analog_input_2 = u.float_in_range(0_f32..=10.23_f32)?;
        let inv_analog_input_1 = u.float_in_range(0_f32..=10.23_f32)?;
        M163AnalogInputVoltages::new(inv_analog_input_6,inv_analog_input_5,inv_analog_input_4,inv_analog_input_3,inv_analog_input_2,inv_analog_input_1).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M162_Temperature_Set_3
///
/// - ID: 162 (0xa2)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M162TemperatureSet3 {
    raw: [u8; 8],
}

impl M162TemperatureSet3 {
    pub const MESSAGE_ID: u32 = 162;
    
    pub const INV_TORQUE_SHUDDER_MIN: f32 = -3276.8_f32;
    pub const INV_TORQUE_SHUDDER_MAX: f32 = 3276.7_f32;
    pub const INV_MOTOR_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_MOTOR_TEMP_MAX: f32 = 3276.7_f32;
    pub const INV_HOT_SPOT_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_HOT_SPOT_TEMP_MAX: f32 = 3276.7_f32;
    pub const INV_COOLANT_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_COOLANT_TEMP_MAX: f32 = 3276.7_f32;
    
    /// Construct new M162_Temperature_Set_3 from values
    pub fn new(inv_torque_shudder: f32, inv_motor_temp: f32, inv_hot_spot_temp: f32, inv_coolant_temp: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_torque_shudder(inv_torque_shudder)?;
        res.set_inv_motor_temp(inv_motor_temp)?;
        res.set_inv_hot_spot_temp(inv_hot_spot_temp)?;
        res.set_inv_coolant_temp(inv_coolant_temp)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Torque_Shudder
    ///
    /// Shudder compensation value of torque
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_torque_shudder(&self) -> f32 {
        self.inv_torque_shudder_raw()
    }
    
    /// Get raw value of INV_Torque_Shudder
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_torque_shudder_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Torque_Shudder
    #[inline(always)]
    pub fn set_inv_torque_shudder(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 162 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Motor_Temp
    ///
    /// Motor Temperature Sensor
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_motor_temp(&self) -> f32 {
        self.inv_motor_temp_raw()
    }
    
    /// Get raw value of INV_Motor_Temp
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_motor_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Motor_Temp
    #[inline(always)]
    pub fn set_inv_motor_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 162 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Hot_Spot_Temp
    ///
    /// Estimated inverter hot spot temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_hot_spot_temp(&self) -> f32 {
        self.inv_hot_spot_temp_raw()
    }
    
    /// Get raw value of INV_Hot_Spot_Temp
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_hot_spot_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Hot_Spot_Temp
    #[inline(always)]
    pub fn set_inv_hot_spot_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 162 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Coolant_Temp
    ///
    /// Estimated Coolant Temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_coolant_temp(&self) -> f32 {
        self.inv_coolant_temp_raw()
    }
    
    /// Get raw value of INV_Coolant_Temp
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_coolant_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Coolant_Temp
    #[inline(always)]
    pub fn set_inv_coolant_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 162 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M162TemperatureSet3 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M162TemperatureSet3 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M162TemperatureSet3")
                .field("inv_torque_shudder", &self.inv_torque_shudder())
                .field("inv_motor_temp", &self.inv_motor_temp())
                .field("inv_hot_spot_temp", &self.inv_hot_spot_temp())
                .field("inv_coolant_temp", &self.inv_coolant_temp())
            .finish()
        } else {
            f.debug_tuple("M162TemperatureSet3").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M162TemperatureSet3 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_torque_shudder = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_motor_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_hot_spot_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_coolant_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        M162TemperatureSet3::new(inv_torque_shudder,inv_motor_temp,inv_hot_spot_temp,inv_coolant_temp).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M161_Temperature_Set_2
///
/// - ID: 161 (0xa1)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M161TemperatureSet2 {
    raw: [u8; 8],
}

impl M161TemperatureSet2 {
    pub const MESSAGE_ID: u32 = 161;
    
    pub const INV_STALL_BURST_MODEL_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_STALL_BURST_MODEL_TEMP_MAX: f32 = 3276.7_f32;
    pub const INV_RTD2_TEMPERATURE_MIN: f32 = -3276.8_f32;
    pub const INV_RTD2_TEMPERATURE_MAX: f32 = 3276.7_f32;
    pub const INV_RTD1_TEMPERATURE_MIN: f32 = -3276.8_f32;
    pub const INV_RTD1_TEMPERATURE_MAX: f32 = 3276.7_f32;
    pub const INV_CONTROL_BOARD_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_CONTROL_BOARD_TEMP_MAX: f32 = 3276.7_f32;
    
    /// Construct new M161_Temperature_Set_2 from values
    pub fn new(inv_stall_burst_model_temp: f32, inv_rtd2_temperature: f32, inv_rtd1_temperature: f32, inv_control_board_temp: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_stall_burst_model_temp(inv_stall_burst_model_temp)?;
        res.set_inv_rtd2_temperature(inv_rtd2_temperature)?;
        res.set_inv_rtd1_temperature(inv_rtd1_temperature)?;
        res.set_inv_control_board_temp(inv_control_board_temp)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Stall_Burst_Model_Temp
    ///
    /// Hottest temperature estimated from the stall burst thermal model feature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_stall_burst_model_temp(&self) -> f32 {
        self.inv_stall_burst_model_temp_raw()
    }
    
    /// Get raw value of INV_Stall_Burst_Model_Temp
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_stall_burst_model_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Stall_Burst_Model_Temp
    #[inline(always)]
    pub fn set_inv_stall_burst_model_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 161 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_RTD2_Temperature
    ///
    /// RTD input 2 (PT1000) Temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_rtd2_temperature(&self) -> f32 {
        self.inv_rtd2_temperature_raw()
    }
    
    /// Get raw value of INV_RTD2_Temperature
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_rtd2_temperature_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_RTD2_Temperature
    #[inline(always)]
    pub fn set_inv_rtd2_temperature(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 161 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_RTD1_Temperature
    ///
    /// RTD input 1 (PT1000) Temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_rtd1_temperature(&self) -> f32 {
        self.inv_rtd1_temperature_raw()
    }
    
    /// Get raw value of INV_RTD1_Temperature
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_rtd1_temperature_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_RTD1_Temperature
    #[inline(always)]
    pub fn set_inv_rtd1_temperature(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 161 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Control_Board_Temp
    ///
    /// Control Board Temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_control_board_temp(&self) -> f32 {
        self.inv_control_board_temp_raw()
    }
    
    /// Get raw value of INV_Control_Board_Temp
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_control_board_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Control_Board_Temp
    #[inline(always)]
    pub fn set_inv_control_board_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 161 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M161TemperatureSet2 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M161TemperatureSet2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M161TemperatureSet2")
                .field("inv_stall_burst_model_temp", &self.inv_stall_burst_model_temp())
                .field("inv_rtd2_temperature", &self.inv_rtd2_temperature())
                .field("inv_rtd1_temperature", &self.inv_rtd1_temperature())
                .field("inv_control_board_temp", &self.inv_control_board_temp())
            .finish()
        } else {
            f.debug_tuple("M161TemperatureSet2").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M161TemperatureSet2 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_stall_burst_model_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_rtd2_temperature = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_rtd1_temperature = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_control_board_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        M161TemperatureSet2::new(inv_stall_burst_model_temp,inv_rtd2_temperature,inv_rtd1_temperature,inv_control_board_temp).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M160_Temperature_Set_1
///
/// - ID: 160 (0xa0)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M160TemperatureSet1 {
    raw: [u8; 8],
}

impl M160TemperatureSet1 {
    pub const MESSAGE_ID: u32 = 160;
    
    pub const INV_GATE_DRIVER_BOARD_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_GATE_DRIVER_BOARD_TEMP_MAX: f32 = 3276.7_f32;
    pub const INV_MODULE_C_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_MODULE_C_TEMP_MAX: f32 = 3276.7_f32;
    pub const INV_MODULE_B_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_MODULE_B_TEMP_MAX: f32 = 3276.7_f32;
    pub const INV_MODULE_A_TEMP_MIN: f32 = -3276.8_f32;
    pub const INV_MODULE_A_TEMP_MAX: f32 = 3276.7_f32;
    
    /// Construct new M160_Temperature_Set_1 from values
    pub fn new(inv_gate_driver_board_temp: f32, inv_module_c_temp: f32, inv_module_b_temp: f32, inv_module_a_temp: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_gate_driver_board_temp(inv_gate_driver_board_temp)?;
        res.set_inv_module_c_temp(inv_module_c_temp)?;
        res.set_inv_module_b_temp(inv_module_b_temp)?;
        res.set_inv_module_a_temp(inv_module_a_temp)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Gate_Driver_Board_Temp
    ///
    /// Gate Driver Board Temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_gate_driver_board_temp(&self) -> f32 {
        self.inv_gate_driver_board_temp_raw()
    }
    
    /// Get raw value of INV_Gate_Driver_Board_Temp
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_gate_driver_board_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Gate_Driver_Board_Temp
    #[inline(always)]
    pub fn set_inv_gate_driver_board_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 160 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Module_C_Temp
    ///
    /// IGBT Module C Temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_module_c_temp(&self) -> f32 {
        self.inv_module_c_temp_raw()
    }
    
    /// Get raw value of INV_Module_C_Temp
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_module_c_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Module_C_Temp
    #[inline(always)]
    pub fn set_inv_module_c_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 160 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Module_B_Temp
    ///
    /// IGBT Module B Temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_module_b_temp(&self) -> f32 {
        self.inv_module_b_temp_raw()
    }
    
    /// Get raw value of INV_Module_B_Temp
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_module_b_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Module_B_Temp
    #[inline(always)]
    pub fn set_inv_module_b_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 160 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Module_A_Temp
    ///
    /// IGBT Module A Temperature
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "temperature:C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_module_a_temp(&self) -> f32 {
        self.inv_module_a_temp_raw()
    }
    
    /// Get raw value of INV_Module_A_Temp
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_module_a_temp_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Module_A_Temp
    #[inline(always)]
    pub fn set_inv_module_a_temp(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 160 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M160TemperatureSet1 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M160TemperatureSet1 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M160TemperatureSet1")
                .field("inv_gate_driver_board_temp", &self.inv_gate_driver_board_temp())
                .field("inv_module_c_temp", &self.inv_module_c_temp())
                .field("inv_module_b_temp", &self.inv_module_b_temp())
                .field("inv_module_a_temp", &self.inv_module_a_temp())
            .finish()
        } else {
            f.debug_tuple("M160TemperatureSet1").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M160TemperatureSet1 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_gate_driver_board_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_module_c_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_module_b_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_module_a_temp = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        M160TemperatureSet1::new(inv_gate_driver_board_temp,inv_module_c_temp,inv_module_b_temp,inv_module_a_temp).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M174_Firmware_Info
///
/// - ID: 174 (0xae)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M174FirmwareInfo {
    raw: [u8; 8],
}

impl M174FirmwareInfo {
    pub const MESSAGE_ID: u32 = 174;
    
    pub const INV_DATE_CODE_YYYY_MIN: u16 = 0_u16;
    pub const INV_DATE_CODE_YYYY_MAX: u16 = 65535_u16;
    pub const INV_DATE_CODE_MMDD_MIN: u16 = 0_u16;
    pub const INV_DATE_CODE_MMDD_MAX: u16 = 65535_u16;
    pub const INV_SW_VERSION_MIN: u16 = 0_u16;
    pub const INV_SW_VERSION_MAX: u16 = 65535_u16;
    pub const INV_PROJECT_CODE_EEP_VER_MIN: u16 = 0_u16;
    pub const INV_PROJECT_CODE_EEP_VER_MAX: u16 = 65535_u16;
    
    /// Construct new M174_Firmware_Info from values
    pub fn new(inv_date_code_yyyy: u16, inv_date_code_mmdd: u16, inv_sw_version: u16, inv_project_code_eep_ver: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_date_code_yyyy(inv_date_code_yyyy)?;
        res.set_inv_date_code_mmdd(inv_date_code_mmdd)?;
        res.set_inv_sw_version(inv_sw_version)?;
        res.set_inv_project_code_eep_ver(inv_project_code_eep_ver)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_DateCode_YYYY
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_date_code_yyyy(&self) -> u16 {
        self.inv_date_code_yyyy_raw()
    }
    
    /// Get raw value of INV_DateCode_YYYY
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_date_code_yyyy_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_DateCode_YYYY
    #[inline(always)]
    pub fn set_inv_date_code_yyyy(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 174 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_DateCode_MMDD
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_date_code_mmdd(&self) -> u16 {
        self.inv_date_code_mmdd_raw()
    }
    
    /// Get raw value of INV_DateCode_MMDD
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_date_code_mmdd_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_DateCode_MMDD
    #[inline(always)]
    pub fn set_inv_date_code_mmdd(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 174 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_SW_Version
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_sw_version(&self) -> u16 {
        self.inv_sw_version_raw()
    }
    
    /// Get raw value of INV_SW_Version
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_sw_version_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_SW_Version
    #[inline(always)]
    pub fn set_inv_sw_version(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 174 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Project_Code_EEP_Ver
    ///
    /// - Min: 0
    /// - Max: 65535
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_project_code_eep_ver(&self) -> u16 {
        self.inv_project_code_eep_ver_raw()
    }
    
    /// Get raw value of INV_Project_Code_EEP_Ver
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_project_code_eep_ver_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of INV_Project_Code_EEP_Ver
    #[inline(always)]
    pub fn set_inv_project_code_eep_ver(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 65535_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 174 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M174FirmwareInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M174FirmwareInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M174FirmwareInfo")
                .field("inv_date_code_yyyy", &self.inv_date_code_yyyy())
                .field("inv_date_code_mmdd", &self.inv_date_code_mmdd())
                .field("inv_sw_version", &self.inv_sw_version())
                .field("inv_project_code_eep_ver", &self.inv_project_code_eep_ver())
            .finish()
        } else {
            f.debug_tuple("M174FirmwareInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M174FirmwareInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_date_code_yyyy = u.int_in_range(0..=65535)?;
        let inv_date_code_mmdd = u.int_in_range(0..=65535)?;
        let inv_sw_version = u.int_in_range(0..=65535)?;
        let inv_project_code_eep_ver = u.int_in_range(0..=65535)?;
        M174FirmwareInfo::new(inv_date_code_yyyy,inv_date_code_mmdd,inv_sw_version,inv_project_code_eep_ver).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M175_Diag_Data_Message
///
/// - ID: 175 (0xaf)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M175DiagDataMessage {
    raw: [u8; 8],
}

impl M175DiagDataMessage {
    pub const MESSAGE_ID: u32 = 175;
    
    pub const INV_DIAG_RUN_FAULTS_HI_MIN: u16 = 0_u16;
    pub const INV_DIAG_RUN_FAULTS_HI_MAX: u16 = 65535_u16;
    pub const INV_DIAG_VQS_CMD_MIN: f32 = -1000_f32;
    pub const INV_DIAG_VQS_CMD_MAX: f32 = 1000_f32;
    pub const INV_DIAG_FW_OUTPUT_MIN: f32 = -2000_f32;
    pub const INV_DIAG_FW_OUTPUT_MAX: f32 = 2000_f32;
    pub const INV_DIAG_IQ_CMD_MIN: f32 = -2000_f32;
    pub const INV_DIAG_IQ_CMD_MAX: f32 = 2000_f32;
    pub const INV_DIAG_IB_MIN: f32 = -2000_f32;
    pub const INV_DIAG_IB_MAX: f32 = 2000_f32;
    pub const INV_DIAG_SIN_USED_MIN: i16 = -5_i16;
    pub const INV_DIAG_SIN_USED_MAX: i16 = 5_i16;
    pub const INV_DIAG_RUN_FAULTS_LO_MIN: u16 = 0_u16;
    pub const INV_DIAG_RUN_FAULTS_LO_MAX: u16 = 65535_u16;
    pub const INV_DIAG_VD_CMD_MIN: f32 = -1000_f32;
    pub const INV_DIAG_VD_CMD_MAX: f32 = 1000_f32;
    pub const INV_DIAG_MOD_INDEX_MIN: f32 = 0_f32;
    pub const INV_DIAG_MOD_INDEX_MAX: f32 = 2_f32;
    pub const INV_DIAG_VDC_MIN: f32 = 0_f32;
    pub const INV_DIAG_VDC_MAX: f32 = 1000_f32;
    pub const INV_DIAG_IA_MIN: f32 = -2000_f32;
    pub const INV_DIAG_IA_MAX: f32 = 2000_f32;
    pub const INV_DIAG_GAMMA_OBSERVER_MIN: f32 = 0_f32;
    pub const INV_DIAG_GAMMA_OBSERVER_MAX: f32 = 360_f32;
    pub const INV_DIAG_PWM_FREQ_MIN: u16 = 0_u16;
    pub const INV_DIAG_PWM_FREQ_MAX: u16 = 24_u16;
    pub const INV_DIAG_VQ_CMD_MIN: f32 = -1000_f32;
    pub const INV_DIAG_VQ_CMD_MAX: f32 = 1000_f32;
    pub const INV_DIAG_ID_CMD_MIN: f32 = -2000_f32;
    pub const INV_DIAG_ID_CMD_MAX: f32 = 2000_f32;
    pub const INV_DIAG_IC_MIN: f32 = -2000_f32;
    pub const INV_DIAG_IC_MAX: f32 = 2000_f32;
    pub const INV_DIAG_COS_USED_MIN: i16 = -5_i16;
    pub const INV_DIAG_COS_USED_MAX: i16 = 5_i16;
    pub const INV_DIAG_GAMMA_RESOLVER_MIN: f32 = 0_f32;
    pub const INV_DIAG_GAMMA_RESOLVER_MAX: f32 = 360_f32;
    pub const INV_DIAG_SEGMENT_MIN: u8 = 0_u8;
    pub const INV_DIAG_SEGMENT_MAX: u8 = 5_u8;
    pub const INV_DIAG_RECORD_MIN: u8 = 0_u8;
    pub const INV_DIAG_RECORD_MAX: u8 = 160_u8;
    
    /// Construct new M175_Diag_Data_Message from values
    pub fn new(inv_diag_segment: u8, inv_diag_record: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_diag_segment(inv_diag_segment)?;
        res.set_inv_diag_record(inv_diag_record)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// Get raw value of INV_Diag_Segment
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_diag_segment_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        signal
    }
    
    pub fn inv_diag_segment(&mut self) -> Result<M175DiagDataMessageInvDiagSegment, CanError> {
        match self.inv_diag_segment_raw() {
            0 => Ok(M175DiagDataMessageInvDiagSegment::M0(M175DiagDataMessageInvDiagSegmentM0{ raw: self.raw })),
            1 => Ok(M175DiagDataMessageInvDiagSegment::M1(M175DiagDataMessageInvDiagSegmentM1{ raw: self.raw })),
            2 => Ok(M175DiagDataMessageInvDiagSegment::M2(M175DiagDataMessageInvDiagSegmentM2{ raw: self.raw })),
            3 => Ok(M175DiagDataMessageInvDiagSegment::M3(M175DiagDataMessageInvDiagSegmentM3{ raw: self.raw })),
            4 => Ok(M175DiagDataMessageInvDiagSegment::M4(M175DiagDataMessageInvDiagSegmentM4{ raw: self.raw })),
            5 => Ok(M175DiagDataMessageInvDiagSegment::M5(M175DiagDataMessageInvDiagSegmentM5{ raw: self.raw })),
            multiplexor => Err(CanError::InvalidMultiplexor { message_id: 175, multiplexor: multiplexor.into() }),
        }
    }
    /// Set value of INV_Diag_Segment
    #[inline(always)]
    fn set_inv_diag_segment(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 5_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 175 });
        }
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// Set value of INV_Diag_Segment
    #[inline(always)]
    pub fn set_m0(&mut self, value: M175DiagDataMessageInvDiagSegmentM0) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_inv_diag_segment(0)?;
        Ok(())
    }
    
    /// Set value of INV_Diag_Segment
    #[inline(always)]
    pub fn set_m1(&mut self, value: M175DiagDataMessageInvDiagSegmentM1) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_inv_diag_segment(1)?;
        Ok(())
    }
    
    /// Set value of INV_Diag_Segment
    #[inline(always)]
    pub fn set_m2(&mut self, value: M175DiagDataMessageInvDiagSegmentM2) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_inv_diag_segment(2)?;
        Ok(())
    }
    
    /// Set value of INV_Diag_Segment
    #[inline(always)]
    pub fn set_m3(&mut self, value: M175DiagDataMessageInvDiagSegmentM3) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_inv_diag_segment(3)?;
        Ok(())
    }
    
    /// Set value of INV_Diag_Segment
    #[inline(always)]
    pub fn set_m4(&mut self, value: M175DiagDataMessageInvDiagSegmentM4) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_inv_diag_segment(4)?;
        Ok(())
    }
    
    /// Set value of INV_Diag_Segment
    #[inline(always)]
    pub fn set_m5(&mut self, value: M175DiagDataMessageInvDiagSegmentM5) -> Result<(), CanError> {
        let b0 = BitArray::<_, LocalBits>::new(self.raw);
        let b1 = BitArray::<_, LocalBits>::new(value.raw);
        self.raw = b0.bitor(b1).into_inner();
        self.set_inv_diag_segment(5)?;
        Ok(())
    }
    
    /// INV_Diag_Record
    ///
    /// - Min: 0
    /// - Max: 160
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_diag_record(&self) -> u8 {
        self.inv_diag_record_raw()
    }
    
    /// Get raw value of INV_Diag_Record
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn inv_diag_record_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of INV_Diag_Record
    #[inline(always)]
    pub fn set_inv_diag_record(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 160_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 175 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M175DiagDataMessage {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M175DiagDataMessage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M175DiagDataMessage")
                .field("inv_diag_record", &self.inv_diag_record())
            .finish()
        } else {
            f.debug_tuple("M175DiagDataMessage").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M175DiagDataMessage {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_diag_segment = u.int_in_range(0..=5)?;
        let inv_diag_record = u.int_in_range(0..=160)?;
        M175DiagDataMessage::new(inv_diag_segment,inv_diag_record).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for multiplexed signal M175_Diag_Data_Message
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum M175DiagDataMessageInvDiagSegment {
    M0(M175DiagDataMessageInvDiagSegmentM0),
    M1(M175DiagDataMessageInvDiagSegmentM1),
    M2(M175DiagDataMessageInvDiagSegmentM2),
    M3(M175DiagDataMessageInvDiagSegmentM3),
    M4(M175DiagDataMessageInvDiagSegmentM4),
    M5(M175DiagDataMessageInvDiagSegmentM5),
}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct M175DiagDataMessageInvDiagSegmentM0 { raw: [u8; 8] }

impl M175DiagDataMessageInvDiagSegmentM0 {
pub fn new() -> Self { Self { raw: [0u8; 8] } }
/// INV_Diag_Sin_Used
///
/// - Min: -5
/// - Max: 5
/// - Unit: "voltage:V"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_sin_used(&self) -> i16 {
    self.inv_diag_sin_used_raw()
}

/// Get raw value of INV_Diag_Sin_Used
///
/// - Start bit: 48
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_sin_used_raw(&self) -> i16 {
    let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    signal
}

/// Set value of INV_Diag_Sin_Used
#[inline(always)]
pub fn set_inv_diag_sin_used(&mut self, value: i16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -5_i16 || 5_i16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
    Ok(())
}

/// INV_Diag_Gamma_Observer
///
/// - Min: 0
/// - Max: 360
/// - Unit: "angle:deg"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_gamma_observer(&self) -> f32 {
    self.inv_diag_gamma_observer_raw()
}

/// Get raw value of INV_Diag_Gamma_Observer
///
/// - Start bit: 32
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_gamma_observer_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Gamma_Observer
#[inline(always)]
pub fn set_inv_diag_gamma_observer(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_f32 || 360_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
    Ok(())
}

/// INV_Diag_Gamma_Resolver
///
/// - Min: 0
/// - Max: 360
/// - Unit: "angle:deg"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_gamma_resolver(&self) -> f32 {
    self.inv_diag_gamma_resolver_raw()
}

/// Get raw value of INV_Diag_Gamma_Resolver
///
/// - Start bit: 16
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_gamma_resolver_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Gamma_Resolver
#[inline(always)]
pub fn set_inv_diag_gamma_resolver(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_f32 || 360_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
    Ok(())
}

}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct M175DiagDataMessageInvDiagSegmentM1 { raw: [u8; 8] }

impl M175DiagDataMessageInvDiagSegmentM1 {
pub fn new() -> Self { Self { raw: [0u8; 8] } }
/// INV_Diag_Ib
///
/// - Min: -2000
/// - Max: 2000
/// - Unit: "current:A"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_ib(&self) -> f32 {
    self.inv_diag_ib_raw()
}

/// Get raw value of INV_Diag_Ib
///
/// - Start bit: 48
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_ib_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Ib
#[inline(always)]
pub fn set_inv_diag_ib(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -2000_f32 || 2000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
    Ok(())
}

/// INV_Diag_Ia
///
/// - Min: -2000
/// - Max: 2000
/// - Unit: "current:A"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_ia(&self) -> f32 {
    self.inv_diag_ia_raw()
}

/// Get raw value of INV_Diag_Ia
///
/// - Start bit: 32
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_ia_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Ia
#[inline(always)]
pub fn set_inv_diag_ia(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -2000_f32 || 2000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
    Ok(())
}

/// INV_Diag_Cos_Used
///
/// - Min: -5
/// - Max: 5
/// - Unit: "voltage:V"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_cos_used(&self) -> i16 {
    self.inv_diag_cos_used_raw()
}

/// Get raw value of INV_Diag_Cos_Used
///
/// - Start bit: 16
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_cos_used_raw(&self) -> i16 {
    let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    signal
}

/// Set value of INV_Diag_Cos_Used
#[inline(always)]
pub fn set_inv_diag_cos_used(&mut self, value: i16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -5_i16 || 5_i16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
    Ok(())
}

}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct M175DiagDataMessageInvDiagSegmentM2 { raw: [u8; 8] }

impl M175DiagDataMessageInvDiagSegmentM2 {
pub fn new() -> Self { Self { raw: [0u8; 8] } }
/// INV_Diag_Iq_cmd
///
/// - Min: -2000
/// - Max: 2000
/// - Unit: "current:A"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_iq_cmd(&self) -> f32 {
    self.inv_diag_iq_cmd_raw()
}

/// Get raw value of INV_Diag_Iq_cmd
///
/// - Start bit: 48
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_iq_cmd_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Iq_cmd
#[inline(always)]
pub fn set_inv_diag_iq_cmd(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -2000_f32 || 2000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
    Ok(())
}

/// INV_Diag_Vdc
///
/// - Min: 0
/// - Max: 1000
/// - Unit: "voltage:V"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_vdc(&self) -> f32 {
    self.inv_diag_vdc_raw()
}

/// Get raw value of INV_Diag_Vdc
///
/// - Start bit: 32
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_vdc_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Vdc
#[inline(always)]
pub fn set_inv_diag_vdc(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_f32 || 1000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
    Ok(())
}

/// INV_Diag_Ic
///
/// - Min: -2000
/// - Max: 2000
/// - Unit: "current:A"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_ic(&self) -> f32 {
    self.inv_diag_ic_raw()
}

/// Get raw value of INV_Diag_Ic
///
/// - Start bit: 16
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_ic_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Ic
#[inline(always)]
pub fn set_inv_diag_ic(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -2000_f32 || 2000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
    Ok(())
}

}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct M175DiagDataMessageInvDiagSegmentM3 { raw: [u8; 8] }

impl M175DiagDataMessageInvDiagSegmentM3 {
pub fn new() -> Self { Self { raw: [0u8; 8] } }
/// INV_Diag_FW_Output
///
/// - Min: -2000
/// - Max: 2000
/// - Unit: "current:A"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_fw_output(&self) -> f32 {
    self.inv_diag_fw_output_raw()
}

/// Get raw value of INV_Diag_FW_Output
///
/// - Start bit: 48
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_fw_output_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_FW_Output
#[inline(always)]
pub fn set_inv_diag_fw_output(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -2000_f32 || 2000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
    Ok(())
}

/// INV_Diag_Mod_Index
///
/// - Min: 0
/// - Max: 2
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_mod_index(&self) -> f32 {
    self.inv_diag_mod_index_raw()
}

/// Get raw value of INV_Diag_Mod_Index
///
/// - Start bit: 32
/// - Signal size: 16 bits
/// - Factor: 0.0001
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_mod_index_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.0001_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Mod_Index
#[inline(always)]
pub fn set_inv_diag_mod_index(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_f32 || 2_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.0001_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
    Ok(())
}

/// INV_Diag_Id_cmd
///
/// - Min: -2000
/// - Max: 2000
/// - Unit: "current:A"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_id_cmd(&self) -> f32 {
    self.inv_diag_id_cmd_raw()
}

/// Get raw value of INV_Diag_Id_cmd
///
/// - Start bit: 16
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_id_cmd_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Id_cmd
#[inline(always)]
pub fn set_inv_diag_id_cmd(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -2000_f32 || 2000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
    Ok(())
}

}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct M175DiagDataMessageInvDiagSegmentM4 { raw: [u8; 8] }

impl M175DiagDataMessageInvDiagSegmentM4 {
pub fn new() -> Self { Self { raw: [0u8; 8] } }
/// INV_Diag_Vqs_Cmd
///
/// - Min: -1000
/// - Max: 1000
/// - Unit: "voltage:V"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_vqs_cmd(&self) -> f32 {
    self.inv_diag_vqs_cmd_raw()
}

/// Get raw value of INV_Diag_Vqs_Cmd
///
/// - Start bit: 48
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_vqs_cmd_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Vqs_Cmd
#[inline(always)]
pub fn set_inv_diag_vqs_cmd(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -1000_f32 || 1000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
    Ok(())
}

/// INV_Diag_Vd_Cmd
///
/// - Min: -1000
/// - Max: 1000
/// - Unit: "voltage:V"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_vd_cmd(&self) -> f32 {
    self.inv_diag_vd_cmd_raw()
}

/// Get raw value of INV_Diag_Vd_Cmd
///
/// - Start bit: 32
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_vd_cmd_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Vd_Cmd
#[inline(always)]
pub fn set_inv_diag_vd_cmd(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -1000_f32 || 1000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
    Ok(())
}

/// INV_Diag_Vq_Cmd
///
/// - Min: -1000
/// - Max: 1000
/// - Unit: "voltage:V"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_vq_cmd(&self) -> f32 {
    self.inv_diag_vq_cmd_raw()
}

/// Get raw value of INV_Diag_Vq_Cmd
///
/// - Start bit: 16
/// - Signal size: 16 bits
/// - Factor: 0.1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Signed
#[inline(always)]
pub fn inv_diag_vq_cmd_raw(&self) -> f32 {
    let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
    
    let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
    let factor = 0.1_f32;
    let offset = 0_f32;
    (signal as f32) * factor + offset
}

/// Set value of INV_Diag_Vq_Cmd
#[inline(always)]
pub fn set_inv_diag_vq_cmd(&mut self, value: f32) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < -1000_f32 || 1000_f32 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    let factor = 0.1_f32;
    let offset = 0_f32;
    let value = ((value - offset) / factor) as i16;
    
    let value = u16::from_ne_bytes(value.to_ne_bytes());
    self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
    Ok(())
}

}

#[derive(Default)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub struct M175DiagDataMessageInvDiagSegmentM5 { raw: [u8; 8] }

impl M175DiagDataMessageInvDiagSegmentM5 {
pub fn new() -> Self { Self { raw: [0u8; 8] } }
/// INV_Diag_Run_Faults_Hi
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_run_faults_hi(&self) -> u16 {
    self.inv_diag_run_faults_hi_raw()
}

/// Get raw value of INV_Diag_Run_Faults_Hi
///
/// - Start bit: 48
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn inv_diag_run_faults_hi_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
    
    signal
}

/// Set value of INV_Diag_Run_Faults_Hi
#[inline(always)]
pub fn set_inv_diag_run_faults_hi(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
    Ok(())
}

/// INV_Diag_Run_Faults_Lo
///
/// - Min: 0
/// - Max: 65535
/// - Unit: ""
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_run_faults_lo(&self) -> u16 {
    self.inv_diag_run_faults_lo_raw()
}

/// Get raw value of INV_Diag_Run_Faults_Lo
///
/// - Start bit: 32
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn inv_diag_run_faults_lo_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
    
    signal
}

/// Set value of INV_Diag_Run_Faults_Lo
#[inline(always)]
pub fn set_inv_diag_run_faults_lo(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 65535_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
    Ok(())
}

/// INV_Diag_PWM_Freq
///
/// - Min: 0
/// - Max: 24
/// - Unit: "frequency:kHz"
/// - Receivers: Vector__XXX
#[inline(always)]
pub fn inv_diag_pwm_freq(&self) -> u16 {
    self.inv_diag_pwm_freq_raw()
}

/// Get raw value of INV_Diag_PWM_Freq
///
/// - Start bit: 16
/// - Signal size: 16 bits
/// - Factor: 1
/// - Offset: 0
/// - Byte order: LittleEndian
/// - Value type: Unsigned
#[inline(always)]
pub fn inv_diag_pwm_freq_raw(&self) -> u16 {
    let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
    
    signal
}

/// Set value of INV_Diag_PWM_Freq
#[inline(always)]
pub fn set_inv_diag_pwm_freq(&mut self, value: u16) -> Result<(), CanError> {
    #[cfg(feature = "range_checked")]
    if value < 0_u16 || 24_u16 < value {
        return Err(CanError::ParameterOutOfRange { message_id: 175 });
    }
    self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
    Ok(())
}

}


/// BMS_Current_Limit
///
/// - ID: 514 (0x202)
/// - Size: 8 bytes
///
/// Sent by BMS
#[derive(Clone, Copy)]
pub struct BmsCurrentLimit {
    raw: [u8; 8],
}

impl BmsCurrentLimit {
    pub const MESSAGE_ID: u32 = 514;
    
    pub const BMS_MAX_CHARGE_CURRENT_MIN: u16 = 0_u16;
    pub const BMS_MAX_CHARGE_CURRENT_MAX: u16 = 1000_u16;
    pub const BMS_MAX_DISCHARGE_CURRENT_MIN: u16 = 0_u16;
    pub const BMS_MAX_DISCHARGE_CURRENT_MAX: u16 = 1000_u16;
    
    /// Construct new BMS_Current_Limit from values
    pub fn new(bms_max_charge_current: u16, bms_max_discharge_current: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_bms_max_charge_current(bms_max_charge_current)?;
        res.set_bms_max_discharge_current(bms_max_discharge_current)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// BMS_Max_Charge_Current
    ///
    /// Maximum charge current from BMS
    ///
    /// - Min: 0
    /// - Max: 1000
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn bms_max_charge_current(&self) -> u16 {
        self.bms_max_charge_current_raw()
    }
    
    /// Get raw value of BMS_Max_Charge_Current
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn bms_max_charge_current_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of BMS_Max_Charge_Current
    #[inline(always)]
    pub fn set_bms_max_charge_current(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 1000_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 514 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// BMS_Max_Discharge_Current
    ///
    /// Maximum discharge current from BMS
    ///
    /// - Min: 0
    /// - Max: 1000
    /// - Unit: "current:A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn bms_max_discharge_current(&self) -> u16 {
        self.bms_max_discharge_current_raw()
    }
    
    /// Get raw value of BMS_Max_Discharge_Current
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn bms_max_discharge_current_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of BMS_Max_Discharge_Current
    #[inline(always)]
    pub fn set_bms_max_discharge_current(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 1000_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 514 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for BmsCurrentLimit {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for BmsCurrentLimit {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("BmsCurrentLimit")
                .field("bms_max_charge_current", &self.bms_max_charge_current())
                .field("bms_max_discharge_current", &self.bms_max_discharge_current())
            .finish()
        } else {
            f.debug_tuple("BmsCurrentLimit").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for BmsCurrentLimit {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let bms_max_charge_current = u.int_in_range(0..=1000)?;
        let bms_max_discharge_current = u.int_in_range(0..=1000)?;
        BmsCurrentLimit::new(bms_max_charge_current,bms_max_discharge_current).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M176_Fast_Info
///
/// - ID: 176 (0xb0)
/// - Size: 8 bytes
/// - Transmitter: INV
///
/// To enable fast message set CAN ACTIVE MSGS HI WORD to 0xFFFE.  Setting to default value of 0xFFFF will disable the fast message.
#[derive(Clone, Copy)]
pub struct M176FastInfo {
    raw: [u8; 8],
}

impl M176FastInfo {
    pub const MESSAGE_ID: u32 = 176;
    
    pub const INV_FAST_DC_BUS_VOLTAGE_MIN: f32 = -3276.8_f32;
    pub const INV_FAST_DC_BUS_VOLTAGE_MAX: f32 = 3276.7_f32;
    pub const INV_FAST_MOTOR_SPEED_MIN: i16 = -32768_i16;
    pub const INV_FAST_MOTOR_SPEED_MAX: i16 = 32767_i16;
    pub const INV_FAST_TORQUE_FEEDBACK_MIN: f32 = -3276.8_f32;
    pub const INV_FAST_TORQUE_FEEDBACK_MAX: f32 = 3276.7_f32;
    pub const INV_FAST_TORQUE_COMMAND_MIN: f32 = -3276.8_f32;
    pub const INV_FAST_TORQUE_COMMAND_MAX: f32 = 32767.7_f32;
    
    /// Construct new M176_Fast_Info from values
    pub fn new(inv_fast_dc_bus_voltage: f32, inv_fast_motor_speed: i16, inv_fast_torque_feedback: f32, inv_fast_torque_command: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_fast_dc_bus_voltage(inv_fast_dc_bus_voltage)?;
        res.set_inv_fast_motor_speed(inv_fast_motor_speed)?;
        res.set_inv_fast_torque_feedback(inv_fast_torque_feedback)?;
        res.set_inv_fast_torque_command(inv_fast_torque_command)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Fast_DC_Bus_Voltage
    ///
    /// DC Bus Voltage
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "voltage:V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_fast_dc_bus_voltage(&self) -> f32 {
        self.inv_fast_dc_bus_voltage_raw()
    }
    
    /// Get raw value of INV_Fast_DC_Bus_Voltage
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_fast_dc_bus_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Fast_DC_Bus_Voltage
    #[inline(always)]
    pub fn set_inv_fast_dc_bus_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 176 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
    /// INV_Fast_Motor_Speed
    ///
    /// Motor speed
    ///
    /// - Min: -32768
    /// - Max: 32767
    /// - Unit: "angular_speed:rpm"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_fast_motor_speed(&self) -> i16 {
        self.inv_fast_motor_speed_raw()
    }
    
    /// Get raw value of INV_Fast_Motor_Speed
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_fast_motor_speed_raw(&self) -> i16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of INV_Fast_Motor_Speed
    #[inline(always)]
    pub fn set_inv_fast_motor_speed(&mut self, value: i16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -32768_i16 || 32767_i16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 176 });
        }
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// INV_Fast_Torque_Feedback
    ///
    /// The estimated torque
    ///
    /// - Min: -3276.8
    /// - Max: 3276.7
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_fast_torque_feedback(&self) -> f32 {
        self.inv_fast_torque_feedback_raw()
    }
    
    /// Get raw value of INV_Fast_Torque_Feedback
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_fast_torque_feedback_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Fast_Torque_Feedback
    #[inline(always)]
    pub fn set_inv_fast_torque_feedback(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 176 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Fast_Torque_Command
    ///
    /// The commanded torque
    ///
    /// - Min: -3276.8
    /// - Max: 32767.7
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_fast_torque_command(&self) -> f32 {
        self.inv_fast_torque_command_raw()
    }
    
    /// Get raw value of INV_Fast_Torque_Command
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_fast_torque_command_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Fast_Torque_Command
    #[inline(always)]
    pub fn set_inv_fast_torque_command(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 32767.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 176 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M176FastInfo {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M176FastInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M176FastInfo")
                .field("inv_fast_dc_bus_voltage", &self.inv_fast_dc_bus_voltage())
                .field("inv_fast_motor_speed", &self.inv_fast_motor_speed())
                .field("inv_fast_torque_feedback", &self.inv_fast_torque_feedback())
                .field("inv_fast_torque_command", &self.inv_fast_torque_command())
            .finish()
        } else {
            f.debug_tuple("M176FastInfo").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M176FastInfo {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_fast_dc_bus_voltage = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_fast_motor_speed = u.int_in_range(-32768..=32767)?;
        let inv_fast_torque_feedback = u.float_in_range(-3276.8_f32..=3276.7_f32)?;
        let inv_fast_torque_command = u.float_in_range(-3276.8_f32..=32767.7_f32)?;
        M176FastInfo::new(inv_fast_dc_bus_voltage,inv_fast_motor_speed,inv_fast_torque_feedback,inv_fast_torque_command).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// M177_Torque_Capability
///
/// - ID: 177 (0xb1)
/// - Size: 8 bytes
/// - Transmitter: INV
#[derive(Clone, Copy)]
pub struct M177TorqueCapability {
    raw: [u8; 8],
}

impl M177TorqueCapability {
    pub const MESSAGE_ID: u32 = 177;
    
    pub const INV_TORQUE_CAPABILITY_REGEN_MIN: f32 = -3276.8_f32;
    pub const INV_TORQUE_CAPABILITY_REGEN_MAX: f32 = 0_f32;
    pub const INV_TORQUE_CAPABILITY_MOTOR_MIN: f32 = 0_f32;
    pub const INV_TORQUE_CAPABILITY_MOTOR_MAX: f32 = 3276.7_f32;
    
    /// Construct new M177_Torque_Capability from values
    pub fn new(inv_torque_capability_regen: f32, inv_torque_capability_motor: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_inv_torque_capability_regen(inv_torque_capability_regen)?;
        res.set_inv_torque_capability_motor(inv_torque_capability_motor)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// INV_Torque_Capability_Regen
    ///
    /// The regen torque capability of the inverter given the current operating point.
    ///
    /// - Min: -3276.8
    /// - Max: 0
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_torque_capability_regen(&self) -> f32 {
        self.inv_torque_capability_regen_raw()
    }
    
    /// Get raw value of INV_Torque_Capability_Regen
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_torque_capability_regen_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Torque_Capability_Regen
    #[inline(always)]
    pub fn set_inv_torque_capability_regen(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -3276.8_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 177 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// INV_Torque_Capability_Motor
    ///
    /// The motoring torque capability of the inverter given the current operating point.
    ///
    /// - Min: 0
    /// - Max: 3276.7
    /// - Unit: "torque:N.m"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn inv_torque_capability_motor(&self) -> f32 {
        self.inv_torque_capability_motor_raw()
    }
    
    /// Get raw value of INV_Torque_Capability_Motor
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn inv_torque_capability_motor_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of INV_Torque_Capability_Motor
    #[inline(always)]
    pub fn set_inv_torque_capability_motor(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 3276.7_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 177 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as i16;
        
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for M177TorqueCapability {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for M177TorqueCapability {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("M177TorqueCapability")
                .field("inv_torque_capability_regen", &self.inv_torque_capability_regen())
                .field("inv_torque_capability_motor", &self.inv_torque_capability_motor())
            .finish()
        } else {
            f.debug_tuple("M177TorqueCapability").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for M177TorqueCapability {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let inv_torque_capability_regen = u.float_in_range(-3276.8_f32..=0_f32)?;
        let inv_torque_capability_motor = u.float_in_range(0_f32..=3276.7_f32)?;
        M177TorqueCapability::new(inv_torque_capability_regen,inv_torque_capability_motor).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Status
///
/// - ID: 2566869221 (0x98ff50e5)
/// - Size: 8 bytes
/// - Transmitter: ElconCharger
#[derive(Clone, Copy)]
pub struct Status {
    raw: [u8; 8],
}

impl Status {
    pub const MESSAGE_ID: u32 = 2566869221;
    
    pub const OUTPUT_CURRENT_MIN: f32 = 0_f32;
    pub const OUTPUT_CURRENT_MAX: f32 = 0_f32;
    pub const OUTPUT_VOLTAGE_MIN: f32 = 0_f32;
    pub const OUTPUT_VOLTAGE_MAX: f32 = 0_f32;
    
    /// Construct new Status from values
    pub fn new(communication_state: bool, state: bool, input_voltage: bool, temperature: bool, hw_status: bool, output_current: f32, output_voltage: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_communication_state(communication_state)?;
        res.set_state(state)?;
        res.set_input_voltage(input_voltage)?;
        res.set_temperature(temperature)?;
        res.set_hw_status(hw_status)?;
        res.set_output_current(output_current)?;
        res.set_output_voltage(output_voltage)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// CommunicationState
    ///
    /// Charger communication state
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn communication_state(&self) -> StatusCommunicationState {
        let signal = self.raw.view_bits::<Lsb0>()[36..37].load_le::<u8>();
        
        match signal {
            0 => StatusCommunicationState::CommStateOk,
            1 => StatusCommunicationState::CommStateTimeout,
            _ => StatusCommunicationState::_Other(self.communication_state_raw()),
        }
    }
    
    /// Get raw value of CommunicationState
    ///
    /// - Start bit: 36
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn communication_state_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[36..37].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of CommunicationState
    #[inline(always)]
    pub fn set_communication_state(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[36..37].store_le(value);
        Ok(())
    }
    
    /// State
    ///
    /// Charger starting state
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn state(&self) -> StatusState {
        let signal = self.raw.view_bits::<Lsb0>()[35..36].load_le::<u8>();
        
        match signal {
            0 => StatusState::StateCharging,
            1 => StatusState::StateOff,
            _ => StatusState::_Other(self.state_raw()),
        }
    }
    
    /// Get raw value of State
    ///
    /// - Start bit: 35
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn state_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[35..36].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of State
    #[inline(always)]
    pub fn set_state(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[35..36].store_le(value);
        Ok(())
    }
    
    /// InputVoltage
    ///
    /// Charger input voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn input_voltage(&self) -> StatusInputVoltage {
        let signal = self.raw.view_bits::<Lsb0>()[34..35].load_le::<u8>();
        
        match signal {
            0 => StatusInputVoltage::InputVoltageOk,
            1 => StatusInputVoltage::InputVoltageFault,
            _ => StatusInputVoltage::_Other(self.input_voltage_raw()),
        }
    }
    
    /// Get raw value of InputVoltage
    ///
    /// - Start bit: 34
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn input_voltage_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[34..35].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of InputVoltage
    #[inline(always)]
    pub fn set_input_voltage(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[34..35].store_le(value);
        Ok(())
    }
    
    /// Temperature
    ///
    /// Charger temperature
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn temperature(&self) -> StatusTemperature {
        let signal = self.raw.view_bits::<Lsb0>()[33..34].load_le::<u8>();
        
        match signal {
            0 => StatusTemperature::TempOk,
            1 => StatusTemperature::TempFault,
            _ => StatusTemperature::_Other(self.temperature_raw()),
        }
    }
    
    /// Get raw value of Temperature
    ///
    /// - Start bit: 33
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temperature_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[33..34].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of Temperature
    #[inline(always)]
    pub fn set_temperature(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[33..34].store_le(value);
        Ok(())
    }
    
    /// HWStatus
    ///
    /// Charger status
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn hw_status(&self) -> StatusHwStatus {
        let signal = self.raw.view_bits::<Lsb0>()[32..33].load_le::<u8>();
        
        match signal {
            0 => StatusHwStatus::HwStatusOk,
            1 => StatusHwStatus::HwStatusFail,
            _ => StatusHwStatus::_Other(self.hw_status_raw()),
        }
    }
    
    /// Get raw value of HWStatus
    ///
    /// - Start bit: 32
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn hw_status_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[32..33].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of HWStatus
    #[inline(always)]
    pub fn set_hw_status(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[32..33].store_le(value);
        Ok(())
    }
    
    /// OutputCurrent
    ///
    /// Output Current
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn output_current(&self) -> f32 {
        self.output_current_raw()
    }
    
    /// Get raw value of OutputCurrent
    ///
    /// - Start bit: 23
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn output_current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[16..32].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of OutputCurrent
    #[inline(always)]
    pub fn set_output_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 2566869221 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[16..32].store_be(value);
        Ok(())
    }
    
    /// OutputVoltage
    ///
    /// Output Voltage
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "V"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn output_voltage(&self) -> f32 {
        self.output_voltage_raw()
    }
    
    /// Get raw value of OutputVoltage
    ///
    /// - Start bit: 7
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn output_voltage_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[0..16].load_be::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of OutputVoltage
    #[inline(always)]
    pub fn set_output_voltage(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 2566869221 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Msb0>()[0..16].store_be(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Status {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Status")
                .field("communication_state", &self.communication_state())
                .field("state", &self.state())
                .field("input_voltage", &self.input_voltage())
                .field("temperature", &self.temperature())
                .field("hw_status", &self.hw_status())
                .field("output_current", &self.output_current())
                .field("output_voltage", &self.output_voltage())
            .finish()
        } else {
            f.debug_tuple("Status").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Status {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let communication_state = u.int_in_range(0..=1)? == 1;
        let state = u.int_in_range(0..=1)? == 1;
        let input_voltage = u.int_in_range(0..=1)? == 1;
        let temperature = u.int_in_range(0..=1)? == 1;
        let hw_status = u.int_in_range(0..=1)? == 1;
        let output_current = u.float_in_range(0_f32..=0_f32)?;
        let output_voltage = u.float_in_range(0_f32..=0_f32)?;
        Status::new(communication_state,state,input_voltage,temperature,hw_status,output_current,output_voltage).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for CommunicationState
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum StatusCommunicationState {
    CommStateOk,
    CommStateTimeout,
    _Other(bool),
}

impl From<StatusCommunicationState> for bool {
    fn from(val: StatusCommunicationState) -> bool {
        match val {
            StatusCommunicationState::CommStateOk => false,
            StatusCommunicationState::CommStateTimeout => true,
            StatusCommunicationState::_Other(x) => x,
        }
    }
}

/// Defined values for State
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum StatusState {
    StateCharging,
    StateOff,
    _Other(bool),
}

impl From<StatusState> for bool {
    fn from(val: StatusState) -> bool {
        match val {
            StatusState::StateCharging => false,
            StatusState::StateOff => true,
            StatusState::_Other(x) => x,
        }
    }
}

/// Defined values for InputVoltage
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum StatusInputVoltage {
    InputVoltageOk,
    InputVoltageFault,
    _Other(bool),
}

impl From<StatusInputVoltage> for bool {
    fn from(val: StatusInputVoltage) -> bool {
        match val {
            StatusInputVoltage::InputVoltageOk => false,
            StatusInputVoltage::InputVoltageFault => true,
            StatusInputVoltage::_Other(x) => x,
        }
    }
}

/// Defined values for Temperature
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum StatusTemperature {
    TempOk,
    TempFault,
    _Other(bool),
}

impl From<StatusTemperature> for bool {
    fn from(val: StatusTemperature) -> bool {
        match val {
            StatusTemperature::TempOk => false,
            StatusTemperature::TempFault => true,
            StatusTemperature::_Other(x) => x,
        }
    }
}

/// Defined values for HWStatus
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum StatusHwStatus {
    HwStatusOk,
    HwStatusFail,
    _Other(bool),
}

impl From<StatusHwStatus> for bool {
    fn from(val: StatusHwStatus) -> bool {
        match val {
            StatusHwStatus::HwStatusOk => false,
            StatusHwStatus::HwStatusFail => true,
            StatusHwStatus::_Other(x) => x,
        }
    }
}



/// This is just to make testing easier
#[allow(dead_code)]
fn main() {}

#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(any(feature = "debug", feature = "std"), derive(Debug))]
pub enum CanError {
    UnknownMessageId(u32),
    /// Signal parameter is not within the range
    /// defined in the dbc
    ParameterOutOfRange {
        /// dbc message id
        message_id: u32,
    },
    InvalidPayloadSize,
    /// Multiplexor value not defined in the dbc
    InvalidMultiplexor {
        /// dbc message id
        message_id: u32,
        /// Multiplexor value not defined in the dbc
        multiplexor: u16,
    },
}

#[cfg(feature = "std")]
use std::error::Error;
#[cfg(feature = "std")]
use std::fmt;

#[cfg(feature = "std")]
impl fmt::Display for CanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[cfg(feature = "std")]
impl Error for CanError {}
#[cfg(feature = "arb")]
trait UnstructuredFloatExt {
    fn float_in_range(&mut self, range: core::ops::RangeInclusive<f32>) -> arbitrary::Result<f32>;
}

#[cfg(feature = "arb")]
impl UnstructuredFloatExt for arbitrary::Unstructured<'_> {
    fn float_in_range(&mut self, range: core::ops::RangeInclusive<f32>) -> arbitrary::Result<f32> {
        let min = range.start();
        let max = range.end();
        let steps = u32::MAX;
        let factor = (max - min) / (steps as f32);
        let random_int: u32 = self.int_in_range(0..=steps)?;
        let random = min + factor * (random_int as f32);
        Ok(random)
    }
}

