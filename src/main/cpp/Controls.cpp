#include "Controls.h"

Controls::Controls(Drivetrain *swerve, Elevator *elevator, Claw *claw, Climber *climber, Pneumatics *pneumatics, Limelight *limelightHigh, Limelight *limelightLow)
{
    // Sets all of the class pointers to the arguments for later use
    this->swerve = swerve;
    this->elevator = elevator;
    this->claw = claw;
    this->climber = climber;
    this->pneumatics = pneumatics;
    this->limelightHigh = limelightHigh;
    this->limelightLow = limelightLow;
}

void Controls::Periodic()
{
    // Runs the different controls
    DriveControls();
    // Must call before elevator and claw controls, otherwise they would be behind 20ms
    SetDesiredPosition();

    if (GetDesiredPosition().has_value() && GetDesiredPosition().value() == Positions::Barge)
    {
        if (!barge.IsScheduled()) barge.Schedule();
    }
    else
    {
        if (barge.IsScheduled()) barge.Cancel();
        ElevatorControls();
        ClawControls();
    }
    ClimberControls();
    PneumaticsControls();
}

void Controls::DriveControls()
{
    if (gamepad.GetYButton())
    {
        swerve->ResetGyro();
    }

    if ((gamepad.GetLeftBumperButtonPressed() && leftPathfindRunnable) || (gamepad.GetRightBumperButtonPressed() && rightPathfindRunnable)) 
    {
        units::meter_t offset = RobotConstants::kRobotLength / 2 + RobotConstants::kBumperWidth;
        Drivetrain::Sides side = Drivetrain::Sides::Right;
        if (gamepad.GetLeftBumperButton())
        {
            side = Drivetrain::Sides::Left;
            leftPathfindRunnable = false;
        }
        else
        {
            rightPathfindRunnable = false;
        }
        try
        {
            path = swerve->PathfindToBranch(side, offset, false);
        
            if (path) path->Schedule();
        }
        catch(...)
        {
            return;
        }
    }
    else if (gamepad.GetAButtonPressed())
    {
        path = swerve->PathfindToProcessor(true);
        if (path) path->Schedule();
    }
    else if (gamepad.GetXButtonPressed())
    {
        path = swerve->PathfindToCoralStation(Drivetrain::Sides::Left, true);
        if (path) path->Schedule();
    }
    else if (gamepad.GetBButtonPressed())
    {
        path = swerve->PathfindToCoralStation(Drivetrain::Sides::Right, true);
        if (path) path->Schedule();
    }
    else if (gamepad.GetLeftBumperButtonReleased() || gamepad.GetRightBumperButtonReleased() 
            || gamepad.GetAButtonReleased()
            || gamepad.GetXButtonReleased() || gamepad.GetBButtonReleased()) 
        { if (path) path->Cancel(); }    


    // Ensures driver control is disabled while pathfinding is occuring
    if (path)
    {
        if (path->IsScheduled()) return;
    }

    if (gamepad.GetPOV() == 0)
    {
        frc::ChassisSpeeds setSpeeds = frc::ChassisSpeeds{0.3_mps, 0.0_mps, 0.0_rad_per_s};
        swerve->Drive(setSpeeds, false);
        return;
    }
    else if (gamepad.GetPOV() == 180)
    {
        frc::ChassisSpeeds setSpeeds = frc::ChassisSpeeds{-0.3_mps, 0.0_mps, 0.0_rad_per_s};
        swerve->Drive(setSpeeds, false);
        return;
    }
    else if (gamepad.GetPOV() == 90)
    {
        frc::ChassisSpeeds setSpeeds = frc::ChassisSpeeds{0.0_mps, -0.3_mps, 0.0_rad_per_s};
        swerve->Drive(setSpeeds, false);
        return;
    }
    else if (gamepad.GetPOV() == 270)
    {
        frc::ChassisSpeeds setSpeeds = frc::ChassisSpeeds{0.0_mps, 0.3_mps, 0.0_rad_per_s};
        swerve->Drive(setSpeeds, false);
        return;
    }

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const double x = -gamepad.GetLeftY();
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const double y = -gamepad.GetLeftX();
    // The scalar helps to smooth out driving while preserving full control over the speed
    const double scalar = x * x + y * y;
    // Adds a speed adjusmtment based on the right trigger - the more it is pressed, the slower the bot will travel for a maximum reduction of -80%
    double speedAdjust = 1 - 0.8 * gamepad.GetRightTriggerAxis();
    speedAdjust -= 0.3 * (elevator->GetHeight() / ElevatorConstants::kMaxElevatorHeight).value();

    const units::meters_per_second_t xSpeed = ApplyDeadband(x * scalar, 0.015) * DrivetrainConstants::kMaxSpeed * speedAdjust;
    const units::meters_per_second_t ySpeed = ApplyDeadband(y * scalar, 0.015) * DrivetrainConstants::kMaxSpeed * speedAdjust;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const double omega = -gamepad.GetRightX();
    const units::radians_per_second_t omegaSpeed = ApplyDeadband(pow(omega, 3), 0.05) *
                     DrivetrainConstants::kMaxAngularSpeed * pow(speedAdjust, 0.5);

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.value());
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.value());
    frc::SmartDashboard::PutNumber("omegaSpeed", omegaSpeed.value());

    if (std::abs(xSpeed.value()) > 0 || std::abs(ySpeed.value()) > 0)
    {
        leftPathfindRunnable = true;
        rightPathfindRunnable = true;
    }
    frc::ChassisSpeeds setSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, omegaSpeed};
    swerve->Drive(setSpeeds, fieldRelative);
}

void Controls::ElevatorControls() 
{
    if (gamepad.GetBackButtonPressed()) elevator->SetBypassTopLimit(false);
    else if (gamepad.GetStartButtonPressed()) elevator->SetBypassTopLimit(true);

    if (GetDesiredPosition().has_value() && frc::SmartDashboard::GetBoolean("Elevator Disable Motion Profiling", false) == false)
    {
        // Runs the elevator to the position
        units::meter_t deltaHeight = GetDesiredPosition().value().height - elevator->GetHeight();
        if ((claw->GetCurrentAngle() <= 13.5_deg && ((deltaHeight > 0_m && elevator->GetHeight() >= 2_ft) || (deltaHeight < 0_m && elevator->GetHeight() <= 2.5_ft))) 
            || (claw->GetCurrentAngle() >= 150_deg && (deltaHeight < 0_m && elevator->GetHeight() < 8_in)))
        {
            elevator->SetMotors(0);   
        }
        else
        {
            bool isElevatorAtPos = elevator->GoToPosition(GetDesiredPosition().value());
            // Set the elevator current position if the height is within the deadzone of the setpoint
            if (isElevatorAtPos == true) SetElevatorPosition(GetDesiredPosition());
            // If the elevator height is not within the deadzone, set the current elevator pos to null
            else SetElevatorPosition(std::nullopt);
        }       
    }  
    else if (controlBoard.GetRawAxis(kManualElevatorAxis) > 0.5) 
    {
        // Manual control up
        elevator->SetMotors(kElevatorPower);
        SetElevatorPosition(std::nullopt);
        elevator->ResetMotionController();
    }
    else if (controlBoard.GetRawAxis(kManualElevatorAxis) < -0.5)
    {
        // Manual control down
        elevator->SetMotors(-kElevatorPower);
        SetElevatorPosition(std::nullopt);
        elevator->ResetMotionController();
    }
    else
    {
        elevator->SetMotors(0.0);
        SetElevatorPosition(std::nullopt);
        elevator->ResetMotionController();
    }
}

void Controls::ClawControls()
{
    if (climber->GetLimit() == true)
    {
        claw->GoToPosition(Positions::CoralHome);
    }
    else if (GetDesiredPosition().has_value() && frc::SmartDashboard::GetBoolean("Wrist Disable Motion Profiling", false) == false)
    {
        units::degree_t desiredAngle = GetDesiredPosition().value().angle;
        units::degree_t deltaAngle = desiredAngle - claw->GetCurrentAngle();
        if (elevator->GetHeight() <= 8_in && (deltaAngle > 0_deg && claw->GetCurrentAngle() >= 150_deg))
        {
            claw->SetWristPower(0.0);
        }
        else
        {
            bool isWristAtPosition = claw->GoToPosition(GetDesiredPosition().value());
            if (isWristAtPosition == true) SetWristPosition(GetDesiredPosition());
            else SetWristPosition(std::nullopt);
        }      
    }
    else if (controlBoard.GetRawAxis(kManualWristAxis) < -0.5)
    {
        claw->SetWristPower(-kWristPower);
        SetWristPosition(std::nullopt);
        claw->ResetMotionController();
    }
    else if (controlBoard.GetRawAxis(kManualWristAxis) > 0.5)
    {
        claw->SetWristPower(kWristPower);
        SetWristPosition(std::nullopt);
        claw->ResetMotionController();
    }
    else
    {
        claw->SetWristPower(0.0);
        SetWristPosition(std::nullopt);
        claw->ResetMotionController();
    }
    
    
    if (controlBoard.GetRawButton(kIOButton))
    {
        if (GetDesiredPosition().has_value())
        {
            // If we are intaking a coral and there the proximity sensor detects a coral, stop the IO motor
            if (GetDesiredPosition().value().isForCoralIntake == true && claw->IsCoralInClaw() == true)
            {
                claw->SetIOPower(0.0);
            }
            // Otherwise, set the IO motor to the constant at the current Position
            else claw->SetIOPower(GetDesiredPosition().value().ioMotorPower);
        }
    }
    else if (controlBoard.GetRawAxis(kManualIntakeAxis) < -0.5) 
    {
        claw->SetIOPower(-kManualIOPower);
    }
    else if (controlBoard.GetRawAxis(kManualIntakeAxis) > 0.5)
    {
        claw->SetIOPower(kManualIOPower);
    }
    else
    {
        claw->SetIOPower(0);
    }
}

void Controls::ClimberControls()
{
    if (controlBoard.GetRawAxis(kClimberAxis) < -0.5)
    {
        climber->SetPower(-1.0);
    }
    else if (controlBoard.GetRawAxis(kClimberAxis) > 0.5)
    {
        climber->SetPower(1.0);
    }
    else
    {
        climber->SetPower(0.0);
    }
}

void Controls::PneumaticsControls()
{
    // pneumatics->SetStopper(controlBoard.GetRawButton(kStopperButton));
}

frc2::CommandPtr Controls::GetBargeCommand()
{
    return frc2::RunCommand
    (
        [this]
        {
            bool isElevatorAtPos = elevator->GoToPosition(Positions::Barge);
            if (isElevatorAtPos == true) SetElevatorPosition(Positions::Barge);
            else SetElevatorPosition(std::nullopt);

            if (Positions::Barge.height - elevator->GetHeight() > 10_in)
            {
                claw->GoToAngle(80_deg);
            }
            else
            {
                claw->GoToAngle(Positions::Barge.angle);
            }
            
            if (Positions::Barge.height - elevator->GetHeight() > 6_in)
            {
                claw->SetIOPower(0.0);
            }
            else
            {
                claw->SetIOPower(kBargePower);
            }
        }
    ).Until
    (
        [this]
        {
            return GetElevatorPosition().has_value();
        }
    );
}

void Controls::SetDesiredPosition()
{
    std::optional<Position> oldPosition = desiredPosition;
    if (controlBoard.GetRawButton(kL1Button))
    {
        desiredPosition = Positions::L1;
    }
    else if (controlBoard.GetRawButton(kL2Button))
    {
        desiredPosition = Positions::L2;
    }
    else if (controlBoard.GetRawButton(kL3Button))
    {
        desiredPosition = Positions::L3;
    }
    else if (controlBoard.GetRawButton(kL4Button))
    {
        desiredPosition = Positions::L4;
    }
    else if (controlBoard.GetRawButton(kAlgaeLowButton))
    {
        desiredPosition = Positions::AlgaeLow;
    }
    else if (controlBoard.GetRawButton(kAlgaeHighButton))
    {
        desiredPosition = Positions::AlgaeHigh;
    }
    else if (controlBoard.GetRawButton(kCoralStationButton))
    {
        desiredPosition = Positions::CoralStation;
    }
    else if (controlBoard.GetRawButton(kProcessorButton))
    {
        desiredPosition = Positions::Processor;
    }
    else if (controlBoard.GetRawButton(kCoralHomeButton))
    {
        desiredPosition = Positions::CoralHome;
    }
    else if (controlBoard.GetRawButton(kAlgaeHomeButton))
    {
        desiredPosition = Positions::AlgaeHome;
    }
    else if (controlBoard.GetRawButton(kBargeButton))
    {
        desiredPosition = Positions::Barge;
    }
    else
    {
        desiredPosition = std::nullopt;
    }

    // if (!oldPosition && desiredPosition)
    // {
    //     elevator->ResetMotionController();
    //     claw->ResetMotionController();
    // }
}

void Controls::UpdateTelemetry()
{
    frc::SmartDashboard::PutString("Wrist Position", GetWristPosition() ? GetWristPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Elevator Position", GetElevatorPosition() ? GetElevatorPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Current Position", GetCurrentPosition() ? GetCurrentPosition().value().to_string() : "None");
    frc::SmartDashboard::PutString("Desired Position", GetDesiredPosition() ? GetDesiredPosition().value().to_string() : "None");
}