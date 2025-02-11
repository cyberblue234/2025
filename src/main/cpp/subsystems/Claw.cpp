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
}

void Claw::SetWristPower(double power)
{
    wristMotor.Set(power);
}

void Claw::GoToAngle(units::degree_t angle)
{
    // Constrains the angle to [-180, 180)
    if (angle < 180_deg && angle >= -180_deg)
    {
        units::turn_t turns = angle * (1_tr / 360_deg);
        controls::PositionVoltage &anglePos = angleOut.WithPosition(turns);
        wristMotor.SetControl(anglePos);
    }
    else
    {
        wristMotor.Set(0);
    }
}

void Claw::GoToPosition(Positions pos)
{
    units::degree_t setAngle = GetAngleToPosition(pos);
    GoToAngle(setAngle);
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
    case Positions::Pickup:
        return kAnglePickup;
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