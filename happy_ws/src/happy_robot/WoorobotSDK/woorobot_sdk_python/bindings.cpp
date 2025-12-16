#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "SerialPortConnector.h"

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
    py::class_<SerialPortConnector>(m, "SerialPortConnector")
    .def(py::init<>())
    .def("get_ports", &SerialPortConnector::getPorts)
    .def("connect_port", &SerialPortConnector::connectPort)
    .def("disconnect", &SerialPortConnector::disconnect)
    .def("add_controller", &SerialPortConnector::addController, py::arg("id"), py::return_value_policy::reference)
    .def_readonly("all_ports", &SerialPortConnector::allPorts)
    .def_readonly("controllers", &SerialPortConnector::controllers);

    py::class_<Controller>(m, "Controller")
    .def(py::init<uint8_t, SerialPortConnector *>(), py::arg("id"), py::arg("serial_port_connector"))
    .def("get_id", &Controller::getId)
    .def("request", &Controller::request, py::arg("id"), py::arg("description"))
    .def("request_all", &Controller::requestAll, py::arg("request"))
    .def("make_request", &Controller::makeRequest, py::arg("request"))
    .def("add_crc16_modbus", &Controller::addCRC16Modbus, py::arg("data"))
    .def("is_crc_valid", &Controller::isCrcValid, py::arg("data"))
    .def("compute_crc16", &Controller::computeCrc16, py::arg("data_blk"), py::arg("data_blk_size"))
    .def("set_baudrate", &Controller::setBaudrate, py::arg("value"))
    .def("get_port_path", &Controller::get_port_path)
    .def("get_current_time_string", &Controller::getCurrentTimeString)
    .def_readonly("board_config", &Controller::boardConfig)
    .def_readonly("main_comm_config", &Controller::mainCommConfig)
    .def_readonly("motor_config", &Controller::motorConfig)
    .def_readonly("motor_param", &Controller::motorParam)
    .def_readonly("motor_gain", &Controller::motorGain)
    .def_readonly("motor_limit", &Controller::motorLimit)
    .def_readonly("motor_status", &Controller::motorStatus)
    .def_readonly("motor_control", &Controller::motorControl)
    .def_readonly("firmware", &Controller::firmware);

    py::class_<BoardConfig>(m, "BoardConfig")
    .def(py::init<Controller *>(), py::arg("controller"))
    .def("software_reset", &BoardConfig::softwareReset)
    .def("location_reset", &BoardConfig::locationReset)
    .def("factory_reset", &BoardConfig::factoryReset)
    .def("set_id", &BoardConfig::setId, py::arg("id"))
    .def("set_error", &BoardConfig::setError, py::arg("value"))
    .def("get_error", &BoardConfig::getError)
    .def("set_max_voltage", &BoardConfig::setMaxVoltage, py::arg("value"))
    .def("get_max_voltage", &BoardConfig::getMaxVoltage)
    .def("set_min_voltage", &BoardConfig::setMinVoltage, py::arg("value"))
    .def("get_min_voltage", &BoardConfig::getMinVoltage)
    .def("set_max_temperature", &BoardConfig::setMaxTemperature, py::arg("value"))
    .def("get_max_temperature", &BoardConfig::getMaxTemperature)
    .def("make_software_reset_command", &BoardConfig::makeSoftwareResetCommand)
    .def("make_location_reset_command", &BoardConfig::makeLocationResetCommand)
    .def("make_factory_reset_command", &BoardConfig::makeFactoryResetCommand)
    .def("make_set_id_command", &BoardConfig::makeSetIdCommand, py::arg("id"))
    .def("make_set_error_command", &BoardConfig::makeSetErrorCommand, py::arg("value"))
    .def("make_get_error_command", &BoardConfig::makeGetErrorCommand)
    .def("make_set_max_voltage_command", &BoardConfig::makeSetMaxVoltageCommand, py::arg("value"))
    .def("make_get_max_voltage_command", &BoardConfig::makeGetMaxVoltageCommand)
    .def("make_set_min_voltage_command", &BoardConfig::makeSetMinVoltageCommand, py::arg("value"))
    .def("make_get_min_voltage_command", &BoardConfig::makeGetMinVoltageCommand)
    .def("make_set_max_temperature_command", &BoardConfig::makeSetMaxTemperatureCommand, py::arg("value"))
    .def("make_get_max_temperature_command", &BoardConfig::makeGetMaxTemperatureCommand)
    .def("set_board_config", &BoardConfig::setBoardConfig,
         py::arg("id"),
         py::arg("error"),
         py::arg("max_voltage"),
         py::arg("min_voltage"),
         py::arg("max_temperature"))

    .def("init", &BoardConfig::init);


    py::class_<MainCommConfig>(m, "MainCommConfig")
    .def(py::init<Controller *>(), py::arg("controller"))
    .def("get_comm_mode", &MainCommConfig::getCommMode)
    .def("set_comm_mode", &MainCommConfig::setCommMode, py::arg("value"))
    .def("get_watchdog", &MainCommConfig::getWatchdog)
    .def("set_watchdog", &MainCommConfig::setWatchdog, py::arg("value"))
    .def("make_get_comm_mode_command", &MainCommConfig::makeGetCommModeCommand)
    .def("make_set_comm_mode_command", &MainCommConfig::makeSetCommModeCommand, py::arg("value"))
    .def("make_get_watchdog_command", &MainCommConfig::makeGetWatchdogCommand)
    .def("make_set_watchdog_command", &MainCommConfig::makeSetWatchdogCommand, py::arg("value"))
    .def("set_main_comm_config", &MainCommConfig::setMainCommConfig,
         py::arg("comm_mode"),
         py::arg("watchdog"))
    .def("init", &MainCommConfig::init);

    py::class_<MotorConfig>(m, "MotorConfig")
    .def(py::init<Controller*>(), py::arg("controller"))
    .def("get_motor_type", &MotorConfig::getMotorType)
    .def("set_motor_type", &MotorConfig::setMotorType, py::arg("value"))
    .def("get_motor_encoder_direction", &MotorConfig::getMotorEncoderDirection)
    .def("set_motor_encoder_direction", &MotorConfig::setMotorEncoderDirection, py::arg("value"))
    .def("get_motor_encoder_cpr", &MotorConfig::getMotorEncoderCpr)
    .def("set_motor_encoder_cpr", &MotorConfig::setMotorEncoderCpr, py::arg("value"))
    .def("get_electronic_angle", &MotorConfig::getElectronicAngle)
    .def("set_electronic_angle", &MotorConfig::setElectronicAngle, py::arg("value"))
    .def("get_gear_ratio", &MotorConfig::getGearRatio)
    .def("set_gear_ratio", &MotorConfig::setGearRatio, py::arg("value"))
    .def("get_lead_angle_mode", &MotorConfig::getLeadAngleMode)
    .def("set_lead_angle_mode", &MotorConfig::setLeadAngleMode, py::arg("value"))
    .def("get_force_angle", &MotorConfig::getForceAngle)
    .def("set_force_angle", &MotorConfig::setForceAngle, py::arg("value"))
    .def("get_electrical_zero", &MotorConfig::getElectricalZero)
    .def("set_electrical_zero", &MotorConfig::setElectricalZero, py::arg("value"))
    .def("get_hall_sensor_set", &MotorConfig::getHallSensorSet)
    .def("set_hall_sensor_set", &MotorConfig::setHallSensorSet, py::arg("value"))
    .def("make_get_motor_type_command", &MotorConfig::makeGetMotorTypeCommand)
    .def("make_set_motor_type_command", &MotorConfig::makeSetMotorTypeCommand, py::arg("value"))
    .def("make_get_motor_encoder_direction_command", &MotorConfig::makeGetMotorEncoderDirectionCommand)
    .def("make_set_motor_encoder_direction_command", &MotorConfig::makeSetMotorEncoderDirectionCommand, py::arg("value"))
    .def("make_get_motor_encoder_cpr_command", &MotorConfig::makeGetMotorEncoderCprCommand)
    .def("make_set_motor_encoder_cpr_command", &MotorConfig::makeSetMotorEncoderCprCommand, py::arg("value"))
    .def("make_get_electronic_angle_command", &MotorConfig::makeGetElectronicAngleCommand)
    .def("make_set_electronic_angle_command", &MotorConfig::makeSetElectronicAngleCommand, py::arg("value"))
    .def("make_get_gear_ratio_command", &MotorConfig::makeGetGearRatioCommand)
    .def("make_set_gear_ratio_command", &MotorConfig::makeSetGearRatioCommand, py::arg("value"))
    .def("make_get_lead_angle_mode_command", &MotorConfig::makeGetLeadAngleModeCommand)
    .def("make_set_lead_angle_mode_command", &MotorConfig::makeSetLeadAngleModeCommand, py::arg("value"))
    .def("make_get_force_angle_command", &MotorConfig::makeGetForceAngleCommand)
    .def("make_set_force_angle_command", &MotorConfig::makeSetForceAngleCommand, py::arg("value"))
    .def("make_get_electrical_zero_command", &MotorConfig::makeGetElectricalZeroCommand)
    .def("make_set_electrical_zero_command", &MotorConfig::makeSetElectricalZeroCommand, py::arg("value"))
    .def("make_get_hall_sensor_set_command", &MotorConfig::makeGetHallSensorSetCommand)
    .def("make_set_hall_sensor_set_command", &MotorConfig::makeSetHallSensorSetCommand, py::arg("value"))
    .def("set_motor_config", &MotorConfig::setMotorConfig,
         py::arg("motor_type"),
         py::arg("encoder_dir"),
         py::arg("encoder_cpr"),
         py::arg("electronic_angle"),
         py::arg("gear_ratio"),
         py::arg("lead_angle_mode"),
         py::arg("force_angle"),
         py::arg("electrical_zero"),
         py::arg("hall_sensor"))
    .def("init", &MotorConfig::init);

     py::class_<MotorControl>(m, "MotorControl")
    .def(py::init<Controller*>(), py::arg("controller"))
    .def("get_acc_time", &MotorControl::getAccTime)
    .def("set_acc_time", &MotorControl::setAccTime, py::arg("value"))
    .def("get_dec_time", &MotorControl::getDecTime)
    .def("set_dec_time", &MotorControl::setDecTime, py::arg("value"))
    .def("get_motion_mode", &MotorControl::getMotionMode)
    .def("set_motion_mode", &MotorControl::setMotionMode, py::arg("value"))
    .def("get_sub_target", &MotorControl::getSubTarget)
    .def("set_sub_target", &MotorControl::setSubTarget, py::arg("value"))
    .def("get_main_target", &MotorControl::getMainTarget)
    .def("set_main_target", &MotorControl::setMainTarget, py::arg("value"))
    .def("get_motor_direction", &MotorControl::getMotorDirection)
    .def("set_motor_direction", &MotorControl::setMotorDirection, py::arg("value"))
    .def("get_run_mode", &MotorControl::getRunMode)
    .def("set_run_mode", &MotorControl::setRunMode, py::arg("value"))
    .def("get_motor_enable", &MotorControl::getMotorEnable)
    .def("set_motor_enable", &MotorControl::setMotorEnable, py::arg("value"))
    .def("get_motor_brake", &MotorControl::getMotorBrake)
    .def("set_motor_brake", &MotorControl::setMotorBrake, py::arg("value"))
    .def("get_external_brake", &MotorControl::getExternalBrake)
    .def("set_external_brake", &MotorControl::setExternalBrake, py::arg("value"))
    .def("make_get_acc_time_command", &MotorControl::makeGetAccTimeCommand)
    .def("make_set_acc_time_command", &MotorControl::makeSetAccTimeCommand, py::arg("value"))
    .def("make_get_dec_time_command", &MotorControl::makeGetDecTimeCommand)
    .def("make_set_dec_time_command", &MotorControl::makeSetDecTimeCommand, py::arg("value"))
    .def("make_get_motion_mode_command", &MotorControl::makeGetMotionModeCommand)
    .def("make_set_motion_mode_command", &MotorControl::makeSetMotionModeCommand, py::arg("value"))
    .def("make_get_sub_target_command", &MotorControl::makeGetSubTargetCommand)
    .def("make_set_sub_target_command", &MotorControl::makeSetSubTargetCommand, py::arg("value"))
    .def("make_get_main_target_command", &MotorControl::makeGetMainTargetCommand)
    .def("make_set_main_target_command", &MotorControl::makeSetMainTargetCommand, py::arg("value"))
    .def("make_get_motor_direction_command", &MotorControl::makeGetMotorDirectionCommand)
    .def("make_set_motor_direction_command", &MotorControl::makeSetMotorDirectionCommand, py::arg("value"))
    .def("make_get_run_mode_command", &MotorControl::makeGetRunModeCommand)
    .def("make_set_run_mode_command", &MotorControl::makeSetRunModeCommand, py::arg("value"))
    .def("make_get_motor_enable_command", &MotorControl::makeGetMotorEnableCommand)
    .def("make_set_motor_enable_command", &MotorControl::makeSetMotorEnableCommand, py::arg("value"))
    .def("make_get_motor_brake_command", &MotorControl::makeGetMotorBrakeCommand)
    .def("make_set_motor_brake_command", &MotorControl::makeSetMotorBrakeCommand, py::arg("value"))
    .def("make_get_external_brake_command", &MotorControl::makeGetExternalBrakeCommand)
    .def("make_set_external_brake_command", &MotorControl::makeSetExternalBrakeCommand, py::arg("value"))
    .def("set_motor_control", &MotorControl::setMotorControl,
         py::arg("acc_time"),
         py::arg("dec_time"),
         py::arg("motion_mode"),
         py::arg("sub_target"),
         py::arg("main_target"),
         py::arg("motor_direction"),
         py::arg("run_mode"),
         py::arg("motor_brake"),
         py::arg("external_brake"))
    .def("init", &MotorControl::init);

     py::class_<MotorGain>(m, "MotorGain")
    .def(py::init<Controller*>(), py::arg("controller"))
    .def("get_position_total_gain", &MotorGain::getPositionTotalGain)
    .def("set_position_total_gain", &MotorGain::setPositionTotalGain, py::arg("value"))
    .def("get_velocity_total_gain", &MotorGain::getVelocityTotalGain)
    .def("set_velocity_total_gain", &MotorGain::setVelocityTotalGain, py::arg("value"))
    .def("get_current_total_gain", &MotorGain::getCurrentTotalGain)
    .def("set_current_total_gain", &MotorGain::setCurrentTotalGain, py::arg("value"))
    .def("get_position_p_gain", &MotorGain::getPositionPGain)
    .def("set_position_p_gain", &MotorGain::setPositionPGain, py::arg("value"))
    .def("get_velocity_p_gain", &MotorGain::getVelocityPGain)
    .def("set_velocity_p_gain", &MotorGain::setVelocityPGain, py::arg("value"))
    .def("get_velocity_i_gain", &MotorGain::getVelocityIGain)
    .def("set_velocity_i_gain", &MotorGain::setVelocityIGain, py::arg("value"))
    .def("get_current_p_gain", &MotorGain::getCurrentPGain)
    .def("set_current_p_gain", &MotorGain::setCurrentPGain, py::arg("value"))
    .def("get_current_i_gain", &MotorGain::getCurrentIGain)
    .def("set_current_i_gain", &MotorGain::setCurrentIGain, py::arg("value"))
    .def("make_get_position_total_gain_command", &MotorGain::makeGetPositionTotalGainCommand)
    .def("make_set_position_total_gain_command", &MotorGain::makeSetPositionTotalGainCommand, py::arg("value"))
    .def("make_get_velocity_total_gain_command", &MotorGain::makeGetVelocityTotalGainCommand)
    .def("make_set_velocity_total_gain_command", &MotorGain::makeSetVelocityTotalGainCommand, py::arg("value"))
    .def("make_get_current_total_gain_command", &MotorGain::makeGetCurrentTotalGainCommand)
    .def("make_set_current_total_gain_command", &MotorGain::makeSetCurrentTotalGainCommand, py::arg("value"))
    .def("make_get_position_p_gain_command", &MotorGain::makeGetPositionPGainCommand)
    .def("make_set_position_p_gain_command", &MotorGain::makeSetPositionPGainCommand, py::arg("value"))
    .def("make_get_velocity_p_gain_command", &MotorGain::makeGetVelocityPGainCommand)
    .def("make_set_velocity_p_gain_command", &MotorGain::makeSetVelocityPGainCommand, py::arg("value"))
    .def("make_get_velocity_i_gain_command", &MotorGain::makeGetVelocityIGainCommand)
    .def("make_set_velocity_i_gain_command", &MotorGain::makeSetVelocityIGainCommand, py::arg("value"))
    .def("make_get_current_p_gain_command", &MotorGain::makeGetCurrentPGainCommand)
    .def("make_set_current_p_gain_command", &MotorGain::makeSetCurrentPGainCommand, py::arg("value"))
    .def("make_get_current_i_gain_command", &MotorGain::makeGetCurrentIGainCommand)
    .def("make_set_current_i_gain_command", &MotorGain::makeSetCurrentIGainCommand, py::arg("value"))
    .def("set_motor_gain", &MotorGain::setMotorGain,
         py::arg("position_total_gain"),
         py::arg("velocity_total_gain"),
         py::arg("current_total_gain"),
         py::arg("position_p_gain"),
         py::arg("velocity_p_gain"),
         py::arg("velocity_i_gain"),
         py::arg("current_p_gain"),
         py::arg("current_i_gain"))
    .def("init", &MotorGain::init);

     py::class_<MotorLimit>(m, "MotorLimit")
    .def(py::init<Controller*>(), py::arg("controller"))
    .def("get_time_max", &MotorLimit::getTimeMax)
    .def("set_time_max", &MotorLimit::setTimeMax, py::arg("value"))
    .def("get_position_ccw_max", &MotorLimit::getPositionCcwMax)
    .def("set_position_ccw_max", &MotorLimit::setPositionCcwMax, py::arg("value"))
    .def("get_position_cw_max", &MotorLimit::getPositionCwMax)
    .def("set_position_cw_max", &MotorLimit::setPositionCwMax, py::arg("value"))
    .def("get_velocity_ccw_max", &MotorLimit::getVelocityCcwMax)
    .def("set_velocity_ccw_max", &MotorLimit::setVelocityCcwMax, py::arg("value"))
    .def("get_velocity_cw_max", &MotorLimit::getVelocityCwMax)
    .def("set_velocity_cw_max", &MotorLimit::setVelocityCwMax, py::arg("value"))
    .def("get_current_ccw_max", &MotorLimit::getCurrentCcwMax)
    .def("set_current_ccw_max", &MotorLimit::setCurrentCcwMax, py::arg("value"))
    .def("get_current_cw_max", &MotorLimit::getCurrentCwMax)
    .def("set_current_cw_max", &MotorLimit::setCurrentCwMax, py::arg("value"))
    .def("get_current_limit", &MotorLimit::getCurrentLimit)
    .def("set_current_limit", &MotorLimit::setCurrentLimit, py::arg("value"))
    .def("make_get_time_max_command", &MotorLimit::makeGetTimeMaxCommand)
    .def("make_set_time_max_command", &MotorLimit::makeSetTimeMaxCommand, py::arg("value"))
    .def("make_get_position_ccw_max_command", &MotorLimit::makeGetPositionCcwMaxCommand)
    .def("make_set_position_ccw_max_command", &MotorLimit::makeSetPositionCcwMaxCommand, py::arg("value"))
    .def("make_get_position_cw_max_command", &MotorLimit::makeGetPositionCwMaxCommand)
    .def("make_set_position_cw_max_command", &MotorLimit::makeSetPositionCwMaxCommand, py::arg("value"))
    .def("make_get_velocity_ccw_max_command", &MotorLimit::makeGetVelocityCcwMaxCommand)
    .def("make_set_velocity_ccw_max_command", &MotorLimit::makeSetVelocityCcwMaxCommand, py::arg("value"))
    .def("make_get_velocity_cw_max_command", &MotorLimit::makeGetVelocityCwMaxCommand)
    .def("make_set_velocity_cw_max_command", &MotorLimit::makeSetVelocityCwMaxCommand, py::arg("value"))
    .def("make_get_current_ccw_max_command", &MotorLimit::makeGetCurrentCcwMaxCommand)
    .def("make_set_current_ccw_max_command", &MotorLimit::makeSetCurrentCcwMaxCommand, py::arg("value"))
    .def("make_get_current_cw_max_command", &MotorLimit::makeGetCurrentCwMaxCommand)
    .def("make_set_current_cw_max_command", &MotorLimit::makeSetCurrentCwMaxCommand, py::arg("value"))
    .def("make_get_current_limit_command", &MotorLimit::makeGetCurrentLimitCommand)
    .def("make_set_current_limit_command", &MotorLimit::makeSetCurrentLimitCommand, py::arg("value"))
    .def("set_motor_limit", &MotorLimit::setMotorLimit,
         py::arg("time_max"),
         py::arg("position_ccw_max"),
         py::arg("position_cw_max"),
         py::arg("velocity_ccw_max"),
         py::arg("velocity_cw_max"),
         py::arg("current_ccw_max"),
         py::arg("current_cw_max"),
         py::arg("current_limit"))
    .def("init", &MotorLimit::init);

     py::class_<MotorParam>(m, "MotorParam")
    .def(py::init<Controller*>(), py::arg("controller"))
    .def("get_d_inductance_constant", &MotorParam::getDInductanceConstant)
    .def("set_d_inductance_constant", &MotorParam::setDInductanceConstant, py::arg("value"))
    .def("get_q_inductance_constant", &MotorParam::getQInductanceConstant)
    .def("set_q_inductance_constant", &MotorParam::setQInductanceConstant, py::arg("value"))
    .def("get_flux_linkage_constant", &MotorParam::getFluxLinkageConstant)
    .def("set_flux_linkage_constant", &MotorParam::setFluxLinkageConstant, py::arg("value"))
    .def("get_resistance_constant", &MotorParam::getResistanceConstant)
    .def("set_resistance_constant", &MotorParam::setResistanceConstant, py::arg("value"))
    .def("get_inductance_constant", &MotorParam::getInductanceConstant)
    .def("set_inductance_constant", &MotorParam::setInductanceConstant, py::arg("value"))
    .def("get_torque_constant", &MotorParam::getTorqueConstant)
    .def("set_torque_constant", &MotorParam::setTorqueConstant, py::arg("value"))
    .def("get_moment_of_inertia", &MotorParam::getMomentOfInertia)
    .def("set_moment_of_inertia", &MotorParam::setMomentOfInertia, py::arg("value"))
    .def("get_rated_speed", &MotorParam::getRatedSpeed)
    .def("set_rated_speed", &MotorParam::setRatedSpeed, py::arg("value"))
    .def("get_rated_current", &MotorParam::getRatedCurrent)
    .def("set_rated_current", &MotorParam::setRatedCurrent, py::arg("value"))
    .def("get_rated_voltage", &MotorParam::getRatedVoltage)
    .def("set_rated_voltage", &MotorParam::setRatedVoltage, py::arg("value"))
    .def("make_get_d_inductance_constant_command", &MotorParam::makeGetDInductanceConstantCommand)
    .def("make_set_d_inductance_constant_command", &MotorParam::makeSetDInductanceConstantCommand, py::arg("value"))
    .def("make_get_q_inductance_constant_command", &MotorParam::makeGetQInductanceConstantCommand)
    .def("make_set_q_inductance_constant_command", &MotorParam::makeSetQInductanceConstantCommand, py::arg("value"))
    .def("make_get_flux_linkage_constant_command", &MotorParam::makeGetFluxLinkageConstantCommand)
    .def("make_set_flux_linkage_constant_command", &MotorParam::makeSetFluxLinkageConstantCommand, py::arg("value"))
    .def("make_get_resistance_constant_command", &MotorParam::makeGetResistanceConstantCommand)
    .def("make_set_resistance_constant_command", &MotorParam::makeSetResistanceConstantCommand, py::arg("value"))
    .def("make_get_inductance_constant_command", &MotorParam::makeGetInductanceConstantCommand)
    .def("make_set_inductance_constant_command", &MotorParam::makeSetInductanceConstantCommand, py::arg("value"))
    .def("make_get_torque_constant_command", &MotorParam::makeGetTorqueConstantCommand)
    .def("make_set_torque_constant_command", &MotorParam::makeSetTorqueConstantCommand, py::arg("value"))
    .def("make_get_moment_of_inertia_command", &MotorParam::makeGetMomentOfInertiaCommand)
    .def("make_set_moment_of_inertia_command", &MotorParam::makeSetMomentOfInertiaCommand, py::arg("value"))
    .def("make_get_rated_speed_command", &MotorParam::makeGetRatedSpeedCommand)
    .def("make_set_rated_speed_command", &MotorParam::makeSetRatedSpeedCommand, py::arg("value"))
    .def("make_get_rated_current_command", &MotorParam::makeGetRatedCurrentCommand)
    .def("make_set_rated_current_command", &MotorParam::makeSetRatedCurrentCommand, py::arg("value"))
    .def("make_get_rated_voltage_command", &MotorParam::makeGetRatedVoltageCommand)
    .def("make_set_rated_voltage_command", &MotorParam::makeSetRatedVoltageCommand, py::arg("value"))
    .def("set_motor_param", &MotorParam::setMotorParam,
         py::arg("d_inductance_constant"),
         py::arg("q_inductance_constant"),
         py::arg("flux_linkage_constant"),
         py::arg("resistance_constant"),
         py::arg("inductance_constant"),
         py::arg("torque_constant"),
         py::arg("moment_of_inertia"),
         py::arg("rated_speed"),
         py::arg("rated_current"),
         py::arg("rated_voltage"))
    .def("init", &MotorParam::init);

     py::class_<MotorStatus>(m, "MotorStatus")
    .def(py::init<Controller*>(), py::arg("controller"))
    .def("get_time_now", &MotorStatus::getTimeNow)
    .def("get_time_left", &MotorStatus::getTimeLeft)
    .def("get_position_now", &MotorStatus::getPositionNow)
    .def("get_position_dir", &MotorStatus::getPositionDir)
    .def("get_velocity_now", &MotorStatus::getVelocityNow)
    .def("get_velocity_dir", &MotorStatus::getVelocityDir)
    .def("get_current_now", &MotorStatus::getCurrentNow)
    .def("get_current_dir", &MotorStatus::getCurrentDir)
    .def("get_encoder_now", &MotorStatus::getEncoderNow)
    .def("get_encoder_dir", &MotorStatus::getEncoderDir)
    .def("get_hallsensor_now", &MotorStatus::getHallsensorNow)
    .def("get_absolute_value_now", &MotorStatus::getAbsoluteValueNow)
    .def("get_voltage_now", &MotorStatus::getVoltageNow)
    .def("get_temperature_now", &MotorStatus::getTemperatureNow)
    .def("get_motor_status", &MotorStatus::getMotorStatus);


    py::class_<Firmware>(m, "Firmware")
    .def(py::init<Controller*>(), py::arg("controller"))
    .def("firmware_update", &Firmware::firmwareUpdate, py::arg("file_path"));
}
