#include "subsystems/Claw.h"

Claw::Claw()
{
    // Starts the configuration process for the wrist motor
    // This line resets any previous configurations to ensure a clean slate
    wristMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration wristMotorConfig{};

    // Stops the motor if there is no input - desirable for ensuring the wrist stays at the desired position
    wristMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    // Configures PID values
    wristMotorConfig.Slot0.kP = kPWrist;
    wristMotorConfig.Slot0.kI = kIWrist;
    wristMotorConfig.Slot0.kD = kDWrist;

    // Sets the CANcoder as the encoder for the wrist motor
    wristMotorConfig.Feedback.FeedbackRemoteSensorID = canCoderWrist.GetDeviceID();
    wristMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;

    // Applies the configuration
    wristMotor.GetConfigurator().Apply(wristMotorConfig);

    canCoderWrist.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderWristConfig{};

    // Sets the offset for the CANcoder - makes sure 0 is when the wrist is horizontal
    canCoderWristConfig.MagnetSensor.MagnetOffset = canCoderMagnetOffset;
    // Sets the range of the CANcoder. When it is at 0.5 turn, the CANcoders range is from [-0.5, 0.5)
    canCoderWristConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;

    canCoderWrist.GetConfigurator().Apply(canCoderWristConfig);

    // Creates the configurator for the IO motor - it's REV, so there are differences in configuration
    SparkBaseConfig ioConfig;
    // Brake mode
    ioConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
    // Limits the current so we don't burn out the motor
    ioConfig.SmartCurrentLimit(60);
    // Applies the configuration, resetting the parameters that it can, and persisting avaiable parameters
    ioMotor.Configure(ioConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    proxSensor.GetConfigurator().Apply(configs::CANrangeConfiguration{});
    configs::CANrangeConfiguration proxSensorConfig{};

    // If an object is detected by the proximity sensor within this value, the .GetIsDetected() will return true
    proxSensorConfig.ProximityParams.ProximityThreshold = 2_in;

    proxSensor.GetConfigurator().Apply(proxSensorConfig);
}

void Claw::SetWristPower(double power)
{
    // Sets the duty cycle of the motor
    wristMotor.Set(power);
}

bool Claw::GoToAngle(units::degree_t angle)
{
    // Constrains the angle to [-180, 180)
    if (angle >= -180_deg && angle < 180_deg)
    {
        // Sets the desired angle
        controls::PositionVoltage &anglePos = angleOut.WithPosition(angle);
        // Tells the motor to go to that position with the PIDs
        wristMotor.SetControl(anglePos.WithSlot(0));
    }
    else
    {
        wristMotor.Set(0);
    }
    // Returns true if the change in angle is less than the deadzone
    return units::math::abs(angle - GetCurrentAngle()) < kDeadzone;
}

bool Claw::GoToPosition(const Position &pos)
{
    return GoToAngle(pos.angle);
}


void Claw::SetIOPower(double power)
{
    // Sets the duty cycle of the motor
    ioMotor.Set(power);
}


void Claw::UpdateTelemetry()
{
    frc::SmartDashboard::PutBoolean("Is Coral in Claw?", IsCoralInClaw());
    frc::SmartDashboard::PutNumber("Proximity Sensor Distance", GetDistance().convert<units::inch>().value());
    frc::SmartDashboard::PutNumber("Wrist Angle", GetCurrentAngle().value());
}

void Claw::SimMode()
{
    ctre::phoenix6::sim::TalonFXSimState& wristMotorSim = wristMotor.GetSimState();
    ctre::phoenix6::sim::CANcoderSimState& canCoderWristSim = canCoderWrist.GetSimState();
    
    wristMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    canCoderWristSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    units::volt_t motorVoltage = wristMotorSim.GetMotorVoltage();

    clawSim.SetInputVoltage(motorVoltage);
    clawSim.Update(20_ms); // assume 20 ms loop time

    wristMotorSim.SetRawRotorPosition(clawSim.GetAngle() * kWristGearRatio.value());
    wristMotorSim.SetRotorVelocity(clawSim.GetVelocity() * kWristGearRatio.value());
    canCoderWristSim.SetRawPosition(clawSim.GetAngle());
    frc::SmartDashboard::PutNumber("Simulated Wrist Angle", clawSim.GetAngle().convert<units::degrees>().value());
}