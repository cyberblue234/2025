#include "subsystems/Claw.h"

Claw::Claw()
{
    wristMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration wristMotorConfig{};

    wristMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    wristMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15_s;
    wristMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15_s;
    wristMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.15_s;

    wristMotorConfig.Slot0.kP = kPWrist;
    wristMotorConfig.Slot0.kI = kIWrist;
    wristMotorConfig.Slot0.kD = kDWrist;

    wristMotorConfig.Feedback.FeedbackRemoteSensorID = canCoderWrist.GetDeviceID();
    wristMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;

    wristMotor.GetConfigurator().Apply(wristMotorConfig);

    canCoderWrist.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderWristConfig{};

    // Sets the offset for the CANcoder - makes sure 0 is when the wrist is horizontal
    canCoderWristConfig.MagnetSensor.MagnetOffset = canCoderMagnetOffset;
    // Sets the range of the CANcoder. When it is at 0.5 turn, the CANcoders range is from [-0.5, 0.5)
    canCoderWristConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5_tr;

    canCoderWrist.GetConfigurator().Apply(canCoderWristConfig);

    SparkBaseConfig intakeConfig;
    intakeConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
    intakeConfig.SmartCurrentLimit(60);
    intakeMotor.Configure(intakeConfig, SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);

    proxSensor.GetConfigurator().Apply(configs::CANrangeConfiguration{});
    configs::CANrangeConfiguration proxSensorConfig{};

    proxSensorConfig.ProximityParams.ProximityThreshold = 2_in;

    proxSensor.GetConfigurator().Apply(proxSensorConfig);
}

void Claw::SetWristPower(double power)
{
    wristMotor.Set(power);
}

bool Claw::GoToAngle(units::degree_t angle)
{
    // Constrains the angle to [-180, 180)
    if (angle >= -180_deg && angle < 180_deg)
    {
        controls::PositionVoltage &anglePos = angleOut.WithPosition(angle);
        wristMotor.SetControl(anglePos);
    }
    else
    {
        wristMotor.Set(0);
    }
    // Returns true if the change in angle is less than the deadzone
    return units::math::abs(angle - GetCurrentAngle()) < kDeadzone;
}

bool Claw::GoToPosition(Positions pos)
{
    units::degree_t setAngle = GetAngleToPosition(pos);
    return GoToAngle(setAngle);
}

units::degree_t Claw::GetAngleToPosition(Positions pos)
{
    switch (pos)
    {
    case Positions::L1:
        return kAngleL1;
        break;
    case Positions::L2:
        return kAngleL2;
        break;
    case Positions::L3:
        return kAngleL3;
        break;
    case Positions::L4:
        return kAngleL4;
        break;
    case Positions::Intake:
        return kAngleIntake;
        break;
    case Positions::Processor:
        return kAngleProcessor;
        break;
    case Positions::Barge:
        return kAngleBarge;
        break;

    default:
        return 0_deg;
        break;
    }
}

void Claw::SetIntakePower(double power)
{
    intakeMotor.Set(power);
}

void Claw::IntakeCoral()
{
    if (IsCoralInClaw() == false)
    {
        SetIntakePower(kCoralIntakePower);
    }
    else
    {
        SetIntakePower(0);
    }
}

void Claw::OutputCoral(Positions pos)
{
    if (IsCoralInClaw() == true)
    {
        if (pos == Positions::L4)
        {
            SetIntakePower(-kCoralIntakePower);
        }
        else
        {
            SetIntakePower(kCoralIntakePower);
        }
    }
    else
    {
        SetIntakePower(0);
    }
}

void Claw::UpdateTelemetry()
{
    frc::SmartDashboard::PutBoolean("Is Coral in Claw?", IsCoralInClaw());
    frc::SmartDashboard::PutNumber("Proximity Sensor Distance", units::inch_t(GetDistance()).value());
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

    wristMotorSim.SetRawRotorPosition(clawSim.GetAngle().convert<units::turn>() / kWristGearRatio.value());
    wristMotorSim.SetRotorVelocity(clawSim.GetVelocity().convert<units::turns_per_second>() / kWristGearRatio.value());
    canCoderWristSim.SetRawPosition(clawSim.GetAngle().convert<units::turn>());
}