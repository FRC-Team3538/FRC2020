/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"
#include <iostream>
#include <frc/Threads.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  IO.drivebase.ResetEncoders();
  IO.drivebase.ResetGyro();
}

void Robot::RobotPeriodic()
{
  // Reset Encoders
  bool btnPSDr = IO.ds.Driver.GetPSButton();
  if (btnPSDr)
  {
    IO.drivebase.ResetEncoders();
    autoPrograms.Init();
  }

  // Update Smart Dash
  UpdateSD();
}

void Robot::AutonomousInit()
{
  IO.drivebase.ResetEncoders();
  IO.drivebase.ResetGyro();
  autoPrograms.Init();
}

void Robot::AutonomousPeriodic()
{
  autoPrograms.Run();
}

void Robot::TeleopInit()
{
}

void Robot::DisabledInit()
{
}

void Robot::TeleopPeriodic()
{

  // Drive
  double forward = Deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand) * -1, deadband);
  double rotate = Deadband(IO.ds.Driver.GetX(GenericHID::kRightHand) * -1, deadband);

  IO.drivebase.Arcade(forward, rotate);

  // Shifting
  if (IO.ds.Driver.GetBumper(GenericHID::kLeftHand))
  {
    IO.drivebase.SetLowGear();
  }

  if (IO.ds.Driver.GetBumper(GenericHID::kRightHand))
  {
    IO.drivebase.SetHighGear();
  }

  //Shooter
  double leftTrigOp = IO.ds.Operator.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigOp = IO.ds.Operator.GetTriggerAxis(GenericHID::kRightHand);
  double leftTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kRightHand);
  double intakeSpeed = leftTrigOp - rightTrigOp + leftTrigDr - rightTrigDr;
  intakeSpeed = Deadband(intakeSpeed, deadband);
  double indexer = 0.0;
  IO.shooter.SetIntake(intakeSpeed);

  if(abs(intakeSpeed) > 0.05)
  {
    indexer = indexerSpeed;
  }

  if(IO.ds.Operator.GetTriangleButton() | IO.ds.Driver.GetTriangleButton())
  {
    IO.shooter.SetShooterDistance(105);
    indexer = indexerSpeed;
  }
  else if(IO.ds.Operator.GetCrossButton() | IO.ds.Driver.GetCrossButton())
  {
    IO.shooter.SetShooterDistance(305);
    indexer = indexerSpeed;
  }
  else
  {
    IO.shooter.SetShooterDistance(0.0);
  }
  
  IO.shooter.SetIndexer(indexer);

  //Hood
  double hoodAnalog = Deadband(IO.ds.Operator.GetY(GenericHID::kRightHand) * -1, deadband);
  IO.shooter.SetHood(hoodAnalog);

  //Climber
  if(IO.ds.Operator.GetBumperPressed(GenericHID::kRightHand))
  {
    IO.climber.ClimberDeploy();
  }
  if(IO.ds.Operator.GetBumperPressed(GenericHID::kLeftHand))
  {
    IO.climber.ClimberRetract();
  }
  double climbAnalog = Deadband(IO.ds.Operator.GetY(GenericHID::kLeftHand) * -1, deadband);
  IO.climber.SetClimber(climbAnalog);
  
  //ColorWheel
   if(IO.ds.Operator.GetUpButton())
  {
    IO.colorWheel.ColorWheelDeploy();
  }
  if(IO.ds.Operator.GetDownButton())
  {
    IO.colorWheel.ColorWheelRetract();
  }
  double colorAnalog = Deadband(IO.ds.Operator.GetX(GenericHID::kRightHand) * -1, deadband);
  IO.colorWheel.SetColorWheel(colorAnalog);

  if(IO.ds.Operator.GetStickButton(GenericHID::kRightHand))
  {
    IO.colorWheel.AutoColorWheel();
  }
}

double Robot::Deadband(double input, double deadband)
{
  if ((std::abs(input)) < deadband)
  {
    return 0.0;
  }
  else if (input > 0.95)
  {
    return 1.0;
  }
  else if (input < -0.95)
  {
    return -1.0;
  }
  else
  {
    return input;
  }
}

void Robot::UpdateSD()
{
  // Don't update smart dash every loop,
  // it causes watchdog warnings
  smartDashSkip %= 30;
  switch (smartDashSkip)
  {
  case 0:
  {
    IO.drivebase.UpdateSmartdash();
    break;
  }

  case 1:
  {
    IO.shooter.UpdateSmartdash();
    break;
  }

  case 2:
  {
    IO.climber.UpdateSmartdash();
    break;
  }

  case 3:
  {
    IO.colorWheel.UpdateSmartdash();
    break;
  }

  }

  // Critical
  autoPrograms.SmartDash();
  IO.ds.SmartDash();
  smartDashSkip++;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
