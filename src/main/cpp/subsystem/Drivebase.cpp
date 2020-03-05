#include "subsystem/Drivebase.hpp"
#include "frc/Timer.h"

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

Drivebase::Drivebase()
{
    // PWM
    motorGroupLeft.SetInverted(false);
    motorGroupRight.SetInverted(true);

    // set default shifter state
    solenoidShifter.Set(false);

    SensorOverride(false);
}

// Arcade Drive
void Drivebase::Arcade(double forward, double turn)
{
    // Constrain input to +/- 1.0
    if (std::abs(forward) > 1.0)
    {
        forward /= std::abs(forward);
    }
    if (std::abs(turn) > 1.0)
    {
        turn /= std::abs(turn);
    }

    // PWM
    motorGroupLeft.Set(forward - turn);
    motorGroupRight.Set(forward + turn);
}

void Drivebase::Tank(double left, double right)
{
    // PWM
    motorGroupLeft.Set(left);
    motorGroupRight.Set(right);
}

void Drivebase::Mecanum(double fwd, double rot, double left)
{
    // PWM
    motorLeft1PWM.Set(-fwd + rot + left);
    motorLeft2PWM.Set(-fwd + rot - left);
    motorRight1PWM.Set(fwd + rot + left);
    motorRight2PWM.Set(fwd + rot - left);
}

// Stop!
void Drivebase::Stop()
{
    
    // PWM
    motorGroupLeft.StopMotor();
    motorGroupRight.StopMotor();
}

// Shift to High Gear
void Drivebase::SetHighGear()
{
    solenoidShifter.Set(true);
}

// Shift to Low Gear
void Drivebase::SetLowGear()
{
    solenoidShifter.Set(false);
}

// Reset Encoders
void Drivebase::ResetEncoders()
{
    
}

double Drivebase::GetEncoderPositionLeft()
{
    return 0.0;
}

double Drivebase::GetEncoderPositionRight()
{
    return 0.0;
}

double Drivebase::GetEncoderPosition()
{
    return 0.0;
}

// Gyro
void Drivebase::ResetGyro()
{
    navx.ZeroYaw();
    forwardHeading = 0;
}

double Drivebase::GetGyroHeading()
{
    double yaw = navx.GetYaw(); 
    return -yaw;
}

void Drivebase::DriveForward(double distance, double currentLimit)
{


    double averageEncCnt = GetEncoderPosition(); 
    double error = distance - averageEncCnt;
    if (error < 24)
    {
        sumError_forward += error / 0.02;
    }
    else
    {
        sumError_forward = 0;
    }
    double deltaError = (error - prevError_forward) / 0.02;
    prevError_forward = error;

    double driveCommandForward = error * KP_FORWARD + sumError_forward * KI_FORWARD + KD_FORWARD * deltaError;

    double gyroAngle = GetGyroHeading(); // -180 ~ 180 

    double errorRot = forwardHeading - gyroAngle;

    if (errorRot > 180.0)
        errorRot -= 360.0;
    if (errorRot < -180.0)
        errorRot += 360.0;

    if (errorRot < 10)
    {
        sumError_rotation += errorRot / 0.02;
    }
    else
    {
        sumError_rotation = 0;
    }
    double deltaErrorRot = errorRot - prevError_rot;
    prevError_rot = error;

    double driveCommandRotation = errorRot * KP_ROTATION + KI_ROTATION * sumError_rotation + KD_ROTATION * deltaErrorRot;

    if (abs(driveCommandRotation) > 0.5)
    {
        if (driveCommandRotation > 0)
        {
            driveCommandRotation = 0.5;
        }
        else
        {
            driveCommandRotation = -0.5;
        }
    }

    Arcade(driveCommandForward, driveCommandRotation);
}

void Drivebase::Turn(double heading)
{
    forwardHeading = heading;
    double errorRot = forwardHeading - GetGyroHeading();
    if (errorRot > 180.0)
        errorRot -= 360.0;
    if (errorRot < -180.0)
        errorRot += 360.0;
    double deltaErrorRot = errorRot - prevError_rot;
    prevError_rot = errorRot;

    double driveCommandRotation = (errorRot * KP_ROTATION) + (KD_ROTATION * deltaErrorRot);

    Arcade(0, driveCommandRotation);
}

void Drivebase::GlobalReset()
{
    oneShotAngle = false;
    prevError_rotation = 0;
    prevError_forward = 0;
    sumError_forward = 0;
    prevError_rot = 0;
}

void Drivebase::SensorOverride(bool active)
{

}

// SmartDash updater
void Drivebase::UpdateSmartdash()
{
    SmartDashboard::PutNumber("DriveCmdL", motorGroupLeft.Get());
    SmartDashboard::PutNumber("DriveCmdR", motorGroupRight.Get());
    
    SmartDashboard::PutBoolean("DriveShifter", solenoidShifter.Get());

    SmartDashboard::PutNumber("DriveEncL", GetEncoderPositionLeft());
    SmartDashboard::PutNumber("DriveEncR", GetEncoderPositionRight());

    SmartDashboard::PutBoolean("DriveOverride", sensorOverride);

    SmartDashboard::PutNumber("Gyro Heading", GetGyroHeading());

    SmartDashboard::PutNumber("Heading Setpoint", forwardHeading);

}
