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
    driveMotorConfig.Slot0.kS = kDrive_kS.value();
    driveMotorConfig.Slot0.kV = kDrive_kV.value();
    driveMotorConfig.Slot0.kA = kDrive_kA.value();

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

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    // Optimize the reference state to avoid spinning further than 90 degrees
    // Deprecated!
    state.Optimize(GetAngle());

    // Scale speed by cosine of angle error. This scales down movement
    // perpendicular to the desired direction of travel that can occur when
    // modules change directions. This results in smoother driving.
    state.CosineScale(GetAngle());

    units::turn_t deltaAngle = units::turn_t(state.angle.operator-(GetAngle()).Degrees().value() / 360);
    // Calculate the turning motor output from the turning PID controller.
    if (frc::RobotBase::IsReal())
    {
        controls::PositionVoltage& turnPos = turnPositionOut.WithPosition(deltaAngle + GetCANcoderPosition());
        turnMotor.SetControl(turnPos);
    }
    else
    {
        ctre::phoenix6::sim::CANcoderSimState& canCoderSim = canCoder.GetSimState();
        canCoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
        canCoderSim.AddPosition(deltaAngle);
    }
    controls::VelocityVoltage& driveVelocity = driveVelocityOut.WithVelocity(state.speed * (1 / kDriveDistanceRatio));
    driveMotor.SetControl(driveVelocity);
    turn_t deltaAngle = state.angle.operator-(GetAngle()).Degrees() / 360;

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


void SwerveModule::SimMode()
{
    ctre::phoenix6::sim::TalonFXSimState& driveMotorSim = driveMotor.GetSimState();

    // set the supply voltage of the TalonFX
    driveMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    // get the motor voltage of the TalonFX
    auto driveMotorVoltage = driveMotorSim.GetMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    driveMotorSimModel.SetInputVoltage(driveMotorVoltage);
    driveMotorSimModel.Update(20_ms); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    driveMotorSim.SetRawRotorPosition(kDriveGearRatio.value() * driveMotorSimModel.GetAngularPosition());
    driveMotorSim.SetRotorVelocity(kDriveGearRatio.value() * driveMotorSimModel.GetAngularVelocity());
    
}