#include "subsystem/Shooter.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
Shooter::Shooter()
{
   flywheel.ConfigFactoryDefault();
   flywheelB.ConfigFactoryDefault();

   flywheelB.Follow(flywheel);

   flywheel.ConfigVoltageCompSaturation(10.0);
   flywheelB.ConfigVoltageCompSaturation(10.0);

   flywheel.Config_kF(0, 0.05643219); //0.05562068
   flywheel.Config_kP(0, 0.25);       //0.35
   flywheel.Config_kI(0, 0.000);      // 0.000
   flywheel.Config_kD(0, 7.000);      //6.0

   flywheel.ConfigClosedloopRamp(0.3);
   flywheelB.ConfigClosedloopRamp(0.3);

   flywheel.SetInverted(false);
   flywheelB.SetInverted(true);

   Test2.Follow(motorIndexer);
   Test.Follow(motorIndexer);
   Test2.SetInverted(false);
   Test.SetInverted(true);
   motorIndexer.SetInverted(true);
   motorFeeder.SetInverted(true);
   motorIntake.SetInverted(true);
   chooseShooterMode.SetDefaultOption(sAutoMode, sAutoMode);
   chooseShooterMode.AddOption(sManualMode, sManualMode);
   hoodEnc.SetDistancePerRotation(360 / 2.5);
}

// Stop all motors
void Shooter::Stop()
{
   flywheel.StopMotor();
   flywheelB.StopMotor();
   motorIntake.StopMotor();
   motorIndexer.StopMotor();
   motorFeeder.StopMotor();
   motorHood.StopMotor();
   shootSpeed = 0.0;
}

void Shooter::StopShooter()
{
   flywheel.StopMotor();
   motorFeeder.StopMotor();
   shootOS = false;
   distOS = false;
}

void Shooter::SetShooterDistanceThree(double distance)
{
   if (!distOS || distance < 0.0)
   {
      dist = distance;
      distOS = true;
   }

   //std::cout << std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) << std::endl;
   if (dist < 0.0)
   {
      motorFeeder.Set(ControlMode::PercentOutput, 0.0);
      shootSpeed = 0.0;
      distOS = false;
   }
   else if (dist < 210)
   {
      shootSpeed = (-0.0135 * pow(dist, 2)) + (7.0252 * dist) + 2073.2; //2098.2
   }
   else
   {
      shootSpeed = (0.0021 * pow(dist, 2)) + (2.3603 * dist) + 2500.8;
      //shootSpeed = (3.7015 * distance) + 2560.3; //2480.3 3.6415
   }
   flywheel.Set(ControlMode::Velocity, -((shootSpeed / kScaleFactorFly) / 600.0));

   if (std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) < 300.0 && (shootSpeed > 100.0))
   {
      shootCounter++;
   }
   else if (std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) > 2500.0 || (shootSpeed < 100.0))
   {
      shootCounter = 0;
   }

   if (shootCounter > 10)
   {
      motorFeeder.Set(ControlMode::PercentOutput, 100.0);
   }
   else
   {
      motorFeeder.Set(ControlMode::PercentOutput, 0.0);
   }
}

void Shooter::SetShooterDistanceTwo(double distance)
{
   if (!distOS || distance < 0.0)
   {
      dist = distance;
      distOS = true;
   }

   //std::cout << std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) << std::endl;
   if (dist < 0.0)
   {
      motorFeeder.Set(ControlMode::PercentOutput, 0.0);
      shootSpeed = 0.0;
      distOS = false;
   }
   else if (dist < 188)
   {
      shootSpeed = (4.2753 * (dist)) + 2041.9; //2041.9
   }
   else
   {
      shootSpeed = (0.0021 * pow(dist, 2)) + (2.3603 * dist) + 2500.8;
      //shootSpeed = (3.7015 * distance) + 2560.3; //2480.3 3.6415
   }
   flywheel.Set(ControlMode::Velocity, -((shootSpeed / kScaleFactorFly) / 600.0));

   if (std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) < 300.0 && (shootSpeed > 100.0))
   {
      shootCounter++;
   }
   else if (std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) > 2500.0 || (shootSpeed < 100.0))
   {
      shootCounter = 0;
   }

   if (shootCounter > 10)
   {
      motorFeeder.Set(ControlMode::PercentOutput, 100.0);
   }
   else
   {
      motorFeeder.Set(ControlMode::PercentOutput, 0.0);
   }
}

void Shooter::SetVelocity(double velocity)
{
   flywheel.Set(ControlMode::Velocity, -((velocity / kScaleFactorFly) / 600.0));
   // if(!shootOS)
   // {
   //    shootOS = true;
   //    shootDelay.Reset();
   //    shootDelay.Start();
   // }
   // if(shootDelay.Get() > 1.5)
   //    motorFeeder.Set(ControlMode::PercentOutput, 100);
   // else
   // {
   //    motorFeeder.Set(ControlMode::PercentOutput, 0);
   // }
}

void Shooter::SetIntake(double speed)
{
   motorIntake.Set(speed);
}

void Shooter::IntakeDeploy()
{
   solenoidIntake.Set(true);
}

void Shooter::IntakeRetract()
{
   solenoidIntake.Set(false);
}

void Shooter::SetIndexer(double speed)
{
   motorIndexer.Set(speed);
}

void Shooter::SetHood(double input)
{
   if((GetHoodAngle() > 70.0) && (input > 0.0))
   {
      motorHood.Set(0.0);
   }
   else if((GetHoodAngle() < 0.0) && (input < 0.0))
   {
      motorHood.Set(0.0);
   }
   else
   {
      motorHood.Set(input);
   }
   
}

void Shooter::SetHoodAngle(double angle)
{
   if(angle > 70.0)
   {
      angle = 70.0;
   }
   else if (angle < 0.0)
   {
      angle = 0.0;
   }
   else
   {
      //YeetYaw
   }
   
   double error = angle - GetHoodAngle();
   //std::cout << error << std::endl;

   if (std::abs(error) < 5.0)
   {
      iAcc += error / 0.02;
   }
   else
   {
      iAcc = 0;
   }

   motorHood.Set(ControlMode::PercentOutput, ((error * kPHood) + (iAcc * kIHood)));
}

double Shooter::GetHoodAngle()
{
   return hoodEnc.GetDistance();
}

bool Shooter::GetModeChooser()
{
   return manualMode;
}

void Shooter::ManualShoot(double inputFly, double inputKick)
{
   flywheel.Set(inputFly);
   motorFeeder.Set(inputKick);
}

void Shooter::UpdateSmartdash()
{
   SmartDashboard::PutData("_ShooterMode", &chooseShooterMode);
   manualMode = (chooseShooterMode.GetSelected() == sManualMode);

   SmartDashboard::PutNumber("Shoot RPM Cmd", shootSpeed);
   SmartDashboard::PutNumber("Shoot RPM Act", flywheel.GetSelectedSensorVelocity() * kScaleFactorFly * 600.0);
   SmartDashboard::PutNumber("Intake", motorIntake.Get());
   SmartDashboard::PutNumber("Indexer", motorIndexer.Get());
   SmartDashboard::PutNumber("Feeder", motorFeeder.Get());
   SmartDashboard::PutNumber("Hood", motorHood.Get());

   SmartDashboard::PutNumber("Shoot Encoder", flywheel.GetSelectedSensorPosition(0));

   SmartDashboard::PutBoolean("Solenoid Intake", solenoidIntake.Get());
   SmartDashboard::PutBoolean("Solenoid Hood", solenoidHood.Get());
}
