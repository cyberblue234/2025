#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(std::string name, int driveMotorID, int turnMotorID, int canCoderID, turn_t canCoderMagnetOffset)
    : driveMotor(driveMotorID, "rio"),
      turnMotor(turnMotorID, "rio"),
      canCoder(canCoderID, "rio")
{
    this->name = name;

    SetEncoder(0_tr);

    driveMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration driveMotorConfig{};

    driveMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15_s;
    driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15_s;
    driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.15_s;


    driveMotorConfig.Slot0.kP = kDriveP;
    driveMotorConfig.Slot0.kI = kDriveI;
    driveMotorConfig.Slot0.kD = kDriveD;

    driveMotor.GetConfigurator().Apply(driveMotorConfig);

    turnMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration turnMotorConfig{};

    turnMotorConfig.Feedback.FeedbackRemoteSensorID = canCoder.GetDeviceID();
    turnMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
    turnMotorConfig.Feedback.SensorToMechanismRatio = 1.0;

    turnMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    turnMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    turnMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15_s;
    turnMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15_s;
    turnMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.15_s;

    turnMotor.GetConfigurator().Apply(turnMotorConfig);

    configs::SlotConfigs turnPIDConfig{};

    turnPIDConfig.kP = kTurnP;
    turnPIDConfig.kI = kTurnI;
    turnPIDConfig.kD = kTurnD;

    turnMotor.GetConfigurator().Apply(turnPIDConfig);

    canCoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderConfig{};

    canCoderConfig.MagnetSensor.MagnetOffset = canCoderMagnetOffset;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1_tr;

    canCoder.GetConfigurator().Apply(canCoderConfig);
}

void SwerveModule::SetDesiredState(SwerveModuleState &state)
{
    // Optimize the reference state to avoid spinning further than 90 degrees
    state.Optimize(GetAngle());
    // Scale speed by cosine of angle error. This scales down movement
    // perpendicular to the desired direction of travel that can occur when
    // modules change directions. This results in smoother driving.
    state.CosineScale(GetAngle());

    turn_t deltaAngle = state.angle.operator-(GetAngle()).Degrees() / 360;
    // Calculate the turning motor output from the turning PID controller.
    controls::PositionVoltage& turnPos = turnPositionOut.WithPosition(deltaAngle + GetCANcoderPosition());
    turnMotor.SetControl(turnPos);
    // Set the motor outputs.
    driveMotor.Set(state.speed.value() / DrivetrainConstants::kMaxSpeed.value());

    TelemetryHelperNumber("SetSpeed", state.speed.value());
    TelemetryHelperNumber("SetAngle", state.angle.Degrees().value());
}

void SwerveModule::UpdateTelemetry()
{
    TelemetryHelperNumber("Distance (m)", GetDistance().value());
    TelemetryHelperNumber("Angle (degrees)", GetAngle().Degrees().value());
    TelemetryHelperNumber("Raw Cancoder", GetAbsoluteCANcoderPosition().value());
    TelemetryHelperNumber("Position Cancoder", GetCANcoderPosition().value());
    TelemetryHelperNumber("Velocity", GetVelocity().value());
    TelemetryHelperNumber("Drive Output Voltage", GetDriveOutputVoltage().value());
    TelemetryHelperNumber("Turn Output Voltage",  GetTurnOutputVoltage().value());
    TelemetryHelperNumber("Drive Torque Current", GetDriveTorqueCurrent().value());
    TelemetryHelperNumber("Turn Torque Current",  GetTurnTorqueCurrent().value());
    TelemetryHelperNumber("Drive Stator Current", GetDriveStatorCurrent().value());
    TelemetryHelperNumber("Turn Stator Current",  GetTurnStatorCurrent().value());
    TelemetryHelperNumber("Drive Supply Current", GetDriveSupplyCurrent().value());
    TelemetryHelperNumber("Turn Supply Current",  GetTurnSupplyCurrent().value());
    TelemetryHelperNumber("Drive Motor Temp", GetDriveTemp().value());
    TelemetryHelperNumber("Turn Motor Temp",  GetTurnTemp().value());
    TelemetryHelperNumber("Drive Processor Temp", GetDriveProcessorTemp().value());
    TelemetryHelperNumber("Turn Processor Temp",  GetTurnProcessorTemp().value());
}