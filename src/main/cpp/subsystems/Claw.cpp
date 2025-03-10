#include "subsystems/Claw.h"

Claw::Claw()
{
    // Starts the configuration process for the wrist motor
    // This line resets any previous configurations to ensure a clean slate
    wristMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration wristMotorConfig{};

    // Stops the motor if there is no input - desirable for ensuring the wrist stays at the desired position
    wristMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    wristMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    // Sets the CANcoder as the encoder for the wrist motor
    wristMotorConfig.Feedback.FeedbackRemoteSensorID = canCoderWrist.GetDeviceID();
    wristMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;

    // Applies the configuration
    wristMotor.GetConfigurator().Apply(wristMotorConfig);

    canCoderWrist.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderWristConfig{};

    // Sets the offset for the CANcoder - makes sure 0 is when the wrist is horizontal
    canCoderWristConfig.MagnetSensor.MagnetOffset = canCoderMagnetOffset;
    // Sets the range of the CANcoder. When it is at 0.5 turn, the CANcoders range is from [-0.2, 0.8)
    canCoderWristConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.8_tr;
    canCoderWristConfig.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;

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

    frc::SmartDashboard::PutNumber("Wrist P", kP);
    frc::SmartDashboard::PutNumber("Wrist I", kI);
    frc::SmartDashboard::PutNumber("Wrist D", kD);
    frc::SmartDashboard::PutNumber("Wrist Trapezoid Max Velocity", kTrapezoidProfileContraints.maxVelocity.value());
    frc::SmartDashboard::PutNumber("Wrist Trapezoid Max Acceleration", kTrapezoidProfileContraints.maxAcceleration.value());
    frc::SmartDashboard::PutNumber("Wrist kS", kS.value());
    frc::SmartDashboard::PutNumber("Wrist kG", kG.value());
    frc::SmartDashboard::PutNumber("Wrist kV", kV.value());
    frc::SmartDashboard::PutNumber("Wrist kA", kA.value());

    frc::SmartDashboard::PutBoolean("Wrist Disable Motion Profiling", false);

    controller.SetTolerance(kTolerance);
}

void Claw::SetWristPower(double power)
{
    // Sets the duty cycle of the motor
    wristMotor.SetControl(controls::DutyCycleOut{power}
                        .WithLimitReverseMotion(GetCurrentAngle() <= kLowLimit)
                        .WithLimitForwardMotion(GetCurrentAngle() >= kHighLimit));
}

bool Claw::GoToAngle(units::degree_t angle)
{
    controller.SetGoal(angle);
    units::volt_t pidSet{controller.Calculate(GetCurrentAngle())};
    units::volt_t feedforwardSet = feedforward.Calculate(GetCurrentAngle(), controller.GetSetpoint().velocity);
    if (frc::SmartDashboard::GetBoolean("Wrist Disable Motion Profiling", false) == false)
    {
        wristMotor.SetControl(voltageOut.WithOutput(pidSet + feedforwardSet)
                            .WithLimitReverseMotion(GetCurrentAngle() <= kLowLimit)
                            .WithLimitForwardMotion(GetCurrentAngle() >= kHighLimit));
    }

    // Returns true if the change in angle is less than the deadzone
    return IsAtPosition();
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
    frc::SmartDashboard::PutNumber("Wrist Setpoint", controller.GetSetpoint().position.value());

    double newP = frc::SmartDashboard::GetNumber("Wrist P", kP);
    if (newP != controller.GetP()) controller.SetP(newP);
    double newI = frc::SmartDashboard::GetNumber("Wrist I", kI);
    if (newI != controller.GetI()) controller.SetI(newI);
    double newD = frc::SmartDashboard::GetNumber("Wrist D", kD);
    if (newD != controller.GetD()) controller.SetD(newD);
    
    double newMaxVel = frc::SmartDashboard::GetNumber("Wrist Trapezoid Max Velocity", kTrapezoidProfileContraints.maxVelocity.value());
    if (newMaxVel != controller.GetConstraints().maxVelocity.value()) 
        controller.SetConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{units::degrees_per_second_t{newMaxVel}, controller.GetConstraints().maxAcceleration});
    double newMaxAccel = frc::SmartDashboard::GetNumber("Wrist Trapezoid Max Acceleration", kTrapezoidProfileContraints.maxAcceleration.value());
    if (newMaxAccel != controller.GetConstraints().maxAcceleration.value()) 
        controller.SetConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{controller.GetConstraints().maxVelocity, units::degrees_per_second_squared_t{newMaxAccel}});

    double newKs = frc::SmartDashboard::GetNumber("Wrist kS", kS.value());
    if (newKs != feedforward.GetKs().value()) feedforward.SetKs(units::volt_t{newKs});
    double newKg = frc::SmartDashboard::GetNumber("Wrist kG", kG.value());
    if (newKg != feedforward.GetKg().value()) feedforward.SetKg(units::volt_t{newKg});
    double newKv = frc::SmartDashboard::GetNumber("Wrist kV", kV.value());
    if (newKv != feedforward.GetKv().value()) feedforward.SetKv(units::kv_degrees_t{newKv});
    double newKa = frc::SmartDashboard::GetNumber("Wrist kA", kA.value());
    if (newKa != feedforward.GetKa().value()) feedforward.SetKa(units::ka_degrees_t{newKa});
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
}