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

    turnMotor.GetConfigurator().Apply(turnPIDConfig);

    canCoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderConfig{};

    // Sets the offset for the CANcoder - makes sure 0 is pointing forward
    canCoderConfig.MagnetSensor.MagnetOffset = canCoderMagnetOffset;
    // Sets the range of the CANcoder. When it is at 1 turn, the CANcoders range is from 0 to 1
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1_tr;

    canCoder.GetConfigurator().Apply(canCoderConfig);

    TelemetryHelperNumber("P", Turn::kP);
    TelemetryHelperNumber("I", Turn::kI);
    TelemetryHelperNumber("D", Turn::kD);
    TelemetryHelperNumber("Trapezoid Max Velocity", Turn::kTrapezoidProfileContraints.maxVelocity.value());
    TelemetryHelperNumber("Trapezoid Max Acceleration", Turn::kTrapezoidProfileContraints.maxAcceleration.value());
    TelemetryHelperNumber("kS", Turn::kS.value());
    TelemetryHelperNumber("kV", Turn::kV.value());
    TelemetryHelperNumber("kA", Turn::kA.value());
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    // Optimize the reference state to avoid spinning further than 90 degrees
    state.Optimize(GetAngle());

    // Scale speed by cosine of angle error. This scales down movement
    // perpendicular to the desired direction of travel that can occur when
    // modules change directions. This results in smoother driving.
    state.CosineScale(GetAngle());

    // Gets the difference between the desired angle and the current angle, then makes it in terms on turns/revolutions
    units::turn_t deltaAngle = units::turn_t(state.angle.operator-(GetAngle()).Degrees().value() / 360);
    
    // Calculate the turning motor output from the turning PID controller.
    // The deltaAngle added to the CANcoder position allows for a clean transition between the CANcoders discontinuity point at 1 turn
    turnController.SetGoal(deltaAngle + GetCANcoderPosition());
    units::volt_t pidSet{turnController.Calculate(GetAngle().Degrees())};
    units::volt_t feedforwardSet = turnFeedforward.Calculate(turnController.GetSetpoint().velocity);
    turnMotor.SetControl(controls::VoltageOut{pidSet + feedforwardSet});
    TelemetryHelperNumber("Turn motor setpoint", turnController.GetSetpoint().position.value());
    TelemetryHelperNumber("Turn motor goal", turnController.GetGoal().position.value());
    
    // Because the motors work based on turns, we have to convert meters to turns, taking into account the gear ratio
    // If the desired speed is 4 meters per second, we can multiply it by turns per meter to get turns per second
    // kDriveDistanceRatio is in meters per turn, but we can use the reciprocal to get turns per meter
    controls::VelocityVoltage& driveVelocity = driveVelocityOut.WithVelocity(state.speed * (1 / kDriveDistanceRatio));
    driveMotor.SetControl(driveVelocity);

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

    double newP = TelemetryHelperGetNumber("P", Turn::kP);
    if (newP != turnController.GetP()) turnController.SetP(newP);
    double newI = TelemetryHelperGetNumber("I", Turn::kI);
    if (newI != turnController.GetI()) turnController.SetI(newI);
    double newD = TelemetryHelperGetNumber("D", Turn::kD);
    if (newD != turnController.GetD()) turnController.SetD(newD);
    
    double newMaxVel = TelemetryHelperGetNumber("Trapezoid Max Velocity", Turn::kTrapezoidProfileContraints.maxVelocity.value());
    if (newMaxVel != turnController.GetConstraints().maxVelocity.value()) 
        turnController.SetConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{units::degrees_per_second_t{newMaxVel}, turnController.GetConstraints().maxAcceleration});
    double newMaxAccel = TelemetryHelperGetNumber("Trapezoid Max Acceleration", Turn::kTrapezoidProfileContraints.maxAcceleration.value());
    if (newMaxAccel != turnController.GetConstraints().maxAcceleration.value()) 
        turnController.SetConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{turnController.GetConstraints().maxVelocity, units::degrees_per_second_squared_t{newMaxAccel}});

    double newKs = TelemetryHelperGetNumber("kS", Turn::kS.value());
    if (newKs != turnFeedforward.GetKs().value()) turnFeedforward.SetKs(units::volt_t{newKs});
    double newKv = TelemetryHelperGetNumber("kV", Turn::kV.value());
    if (newKv != turnFeedforward.GetKv().value()) turnFeedforward.SetKv(units::kv_degrees_t{newKv});
    double newKa = TelemetryHelperGetNumber("kA", Turn::kA.value());
    if (newKa != turnFeedforward.GetKa().value()) turnFeedforward.SetKa(units::ka_degrees_t{newKa});
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

    ctre::phoenix6::sim::TalonFXSimState& turnMotorSim = turnMotor.GetSimState();
    ctre::phoenix6::sim::CANcoderSimState& canCoderSim = canCoder.GetSimState();
    turnMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    canCoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    // get the motor voltage of the TalonFX
    units::volt_t turnMotorVoltage = turnMotorSim.GetMotorVoltage();

    turnMotorSimModel.SetInputVoltage(turnMotorVoltage);
    turnMotorSimModel.Update(20_ms); // assume 20 ms loop time

    canCoderSim.SetRawPosition(turnMotorSimModel.GetAngularPosition());
    canCoderSim.SetVelocity(turnMotorSimModel.GetAngularVelocity());
    turnMotorSim.SetRawRotorPosition(kTurnGearRatio.value() * turnMotorSimModel.GetAngularPosition());
    turnMotorSim.SetRotorVelocity(kTurnGearRatio.value() * turnMotorSimModel.GetAngularVelocity());
}