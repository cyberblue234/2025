#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(std::string name, int driveMotorID, int turnMotorID, int canCoderID, units::turn_t canCoderMagnetOffset)
    : driveMotor(driveMotorID, "rio"),
      turnMotor(turnMotorID, "rio"),
      canCoder(canCoderID, "rio")
{
    // Sets the class name to the parameter name
    this->name = name;

    // Resets the drive encoders
    SetEncoder(0_tr);

    // Starts the configuration process for the drive motors
    // This line resets any previous configurations to ensure a clean slate
    driveMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration driveMotorConfig{};

    // Stops the motor if there is no input - desirable for stopping
    driveMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    // Stator limit makes sure we don't burn up our motors if they get jammed
    driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    // Configures PID and feedforward values
    driveMotorConfig.Slot0.kP = Drive::kP;
    driveMotorConfig.Slot0.kI = Drive::kI;
    driveMotorConfig.Slot0.kD = Drive::kD;
    driveMotorConfig.Slot0.kS = Drive::kS.value();
    driveMotorConfig.Slot0.kV = Drive::kV.value();
    driveMotorConfig.Slot0.kA = Drive::kA.value();

    // Actually applies the configuration
    driveMotor.GetConfigurator().Apply(driveMotorConfig);

    turnMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration turnMotorConfig{};

    // Sets the CANcoder as the encoder for the turn motor
    turnMotorConfig.Feedback.FeedbackRemoteSensorID = canCoder.GetDeviceID();
    turnMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;

    // Positive power to the turn motor will make it go clockwise - vice-versa for negative power
    turnMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    turnMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0_A;

    turnMotor.GetConfigurator().Apply(turnMotorConfig);

    configs::SlotConfigs turnPIDConfig{};

    turnPIDConfig.kP = Turn::kP;
    turnPIDConfig.kI = Turn::kI;
    turnPIDConfig.kD = Turn::kD;

    turnMotor.GetConfigurator().Apply(turnPIDConfig);

    canCoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderConfig{};

    // Sets the offset for the CANcoder - makes sure 0 is pointing forward
    canCoderConfig.MagnetSensor.MagnetOffset = canCoderMagnetOffset;
    // Sets the range of the CANcoder. When it is at 1 turn, the CANcoders range is from 0 to 1
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1_tr;

    canCoder.GetConfigurator().Apply(canCoderConfig);

    if (frc::RobotBase::IsSimulation()) SetCanCoder(0_tr);

    frc::SmartDashboard::PutNumber("Drive kS", Drive::kS.value());
    frc::SmartDashboard::PutNumber("Drive kV", Drive::kV.value());
    frc::SmartDashboard::PutNumber("Drive kA", Drive::kA.value());

    frc::SmartDashboard::PutNumber("Turn P", Turn::kP);
    frc::SmartDashboard::PutNumber("Turn I", Turn::kI);
    frc::SmartDashboard::PutNumber("Turn D", Turn::kD);
    frc::SmartDashboard::PutNumber("Turn Trapezoid Max Velocity", Turn::kTrapezoidProfileContraints.maxVelocity.value());
    frc::SmartDashboard::PutNumber("Turn Trapezoid Max Acceleration", Turn::kTrapezoidProfileContraints.maxAcceleration.value());
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    // Optimize the reference state to avoid spinning further than 90 degrees
    state.Optimize(GetAngle());

    // Scale speed by cosine of angle error. This scales down movement
    // perpendicular to the desired direction of travel that can occur when
    // modules change directions. This results in smoother driving.
    state.CosineScale(GetAngle());

    // Calculate the turning motor output from the turning PID controller.
    // The deltaAngle added to the CANcoder position allows for a clean transition between the CANcoders discontinuity point at 1 turn
    if (frc::RobotBase::IsReal())
    {
        turnController.SetGoal(state.angle.Degrees());
        units::volt_t turnOutput{turnController.Calculate(GetAngle().Degrees())};
        turnMotor.SetControl(controls::VoltageOut{turnOutput});
    }
    else
    {
        // Gets the difference between the desired angle and the current angle, then makes it in terms on turns/revolutions
        units::turn_t deltaAngle = state.angle.operator-(GetAngle()).Degrees();

        ctre::phoenix6::sim::CANcoderSimState& canCoderSim = canCoder.GetSimState();
        canCoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

        canCoderSim.AddPosition(deltaAngle);
    }
    
    driveMotor.SetControl(controls::VoltageOut{driveFeedforward.Calculate(state.speed)});

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

    double newP = frc::SmartDashboard::GetNumber("Turn P", Turn::kP);
    if (newP != turnController.GetP()) turnController.SetP(newP);
    double newI = frc::SmartDashboard::GetNumber("Turn I", Turn::kI);
    if (newI != turnController.GetI()) turnController.SetI(newI);
    double newD = frc::SmartDashboard::GetNumber("Turn D", Turn::kD);
    if (newD != turnController.GetD()) turnController.SetD(newD);
    
    double newMaxVel = frc::SmartDashboard::GetNumber("Turn Trapezoid Max Velocity", Turn::kTrapezoidProfileContraints.maxVelocity.value());
    if (newMaxVel != turnController.GetConstraints().maxVelocity.value()) 
        turnController.SetConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{units::degrees_per_second_t{newMaxVel}, turnController.GetConstraints().maxAcceleration});
    double newMaxAccel = frc::SmartDashboard::GetNumber("Turn Trapezoid Max Acceleration", Turn::kTrapezoidProfileContraints.maxAcceleration.value());
    if (newMaxAccel != turnController.GetConstraints().maxAcceleration.value()) 
        turnController.SetConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{turnController.GetConstraints().maxVelocity, units::degrees_per_second_squared_t{newMaxAccel}});

    double newKs = frc::SmartDashboard::GetNumber("Drive kS", Drive::kS.value());
    if (newKs != driveFeedforward.GetKs().value()) driveFeedforward.SetKs(units::volt_t{newKs});
    double newKv = frc::SmartDashboard::GetNumber("Drive kV", Drive::kV.value());
    if (newKv != driveFeedforward.GetKv().value()) driveFeedforward.SetKv(units::kv_meters_t{newKv});
    double newKa = frc::SmartDashboard::GetNumber("Drive kA", Drive::kA.value());
    if (newKa != driveFeedforward.GetKa().value()) driveFeedforward.SetKa(units::ka_meters_t{newKa});
}


void SwerveModule::SimMode()
{
    ctre::phoenix6::sim::TalonFXSimState& driveMotorSim = driveMotor.GetSimState();

    // set the supply voltage of the TalonFX
    driveMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    // get the motor voltage of the TalonFX
    units::volt_t driveMotorVoltage = driveMotorSim.GetMotorVoltage();

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