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

  
   SmartDashboard::PutNumber("Rotate Max", rotateMax);
   SmartDashboard::SetPersistent("Rotate Max");
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
  double forward = Deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand) * -1.0, deadband);
  double rotate = Deadband(IO.ds.Driver.GetX(GenericHID::kRightHand) * -1.0 * rotateMax, deadband);

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

  // Manip
  double leftTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kRightHand);
  IO.manip.SetA(leftTrigDr - rightTrigDr);
  IO.manip.SetB(leftTrigDr - rightTrigDr);

  if (IO.ds.Driver.GetCrossButton() | IO.ds.Operator.GetCrossButton())
  {
    IO.manip.SetC(1.0);
    IO.manip.SetD(1.0);
  }

  if ( IO.ds.Driver.GetCircleButton() |  IO.ds.Operator.GetCircleButton())
  {
    IO.manip.SetC(0.0);
    IO.manip.SetD(0.0);
  }
  
  // Solenoids
  IO.manip.SetSol1(IO.ds.Driver.GetSquareButton() | IO.ds.Operator.GetSquareButton());

  if (IO.ds.Driver.GetTriangleButton() | IO.ds.Operator.GetTriangleButton())
  {
    IO.manip.ToggleSol2();
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
  case 5:
  {
    IO.manip.UpdateSmartdash();
    break;
  }
  case 10:
  {
    rotateMax = SmartDashboard::GetNumber("Rotate Max", rotateMax);
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
