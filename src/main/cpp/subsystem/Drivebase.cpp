#include "subsystem/Drivebase.hpp"
#include "frc/Timer.h"

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

Drivebase::Drivebase()
{
    // PWM

    // CTRE CAN
    motorLeft1.ConfigFactoryDefault();
    motorLeft2.ConfigFactoryDefault();
    motorLeft3.ConfigFactoryDefault();
    motorRight1.ConfigFactoryDefault();
    motorRight2.ConfigFactoryDefault();
    motorRight3.ConfigFactoryDefault();

    // Invert one side of the drive
    motorLeft1.SetInverted(false);
    motorLeft2.SetInverted(false);
    motorLeft3.SetInverted(false);

    motorRight1.SetInverted(true);
    motorRight2.SetInverted(true);
    motorRight3.SetInverted(true);

    // Brake / Coast Mode
    motorLeft1.SetNeutralMode(NeutralMode::Brake);
    motorLeft2.SetNeutralMode(NeutralMode::Brake);
    motorLeft3.SetNeutralMode(NeutralMode::Brake);
    motorRight1.SetNeutralMode(NeutralMode::Brake);
    motorRight2.SetNeutralMode(NeutralMode::Brake);
    motorRight3.SetNeutralMode(NeutralMode::Brake);

    // set default shifter state
    solenoidShifter.Set(false);

    SensorOverride(false);

    // Encoder Feedback
    motorLeft1.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PIDind::primary);
    motorLeft2.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PIDind::primary);
    motorLeft3.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PIDind::primary);
    motorLeft1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    motorLeft2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    motorLeft3.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    motorRight1.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PIDind::primary);
    motorRight2.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PIDind::primary);
    motorRight3.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PIDind::primary);
    motorRight1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    motorRight2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    motorRight3.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    motorLeft1.SetSensorPhase(false);
    motorLeft2.SetSensorPhase(false);
    motorLeft3.SetSensorPhase(false);
    motorRight1.SetSensorPhase(false);
    motorRight2.SetSensorPhase(false);
    motorRight3.SetSensorPhase(false);

    motorLeft1.Config_kF(slots::Forward, 0.0);
    motorLeft1.Config_kP(slots::Forward, 0.1);
    motorLeft1.Config_kI(slots::Forward, 0.0);
    motorLeft1.Config_kD(slots::Forward, 0.04);

    motorLeft2.Config_kF(slots::Forward, 0.0);
    motorLeft2.Config_kP(slots::Forward, 0.1);
    motorLeft2.Config_kI(slots::Forward, 0.0);
    motorLeft2.Config_kD(slots::Forward, 0.04);

    motorLeft3.Config_kF(slots::Forward, 0.0);
    motorLeft3.Config_kP(slots::Forward, 0.1);
    motorLeft3.Config_kI(slots::Forward, 0.0);
    motorLeft3.Config_kD(slots::Forward, 0.04);

    motorRight1.Config_kF(slots::Forward, 0.0);
    motorRight1.Config_kP(slots::Forward, 0.1);
    motorRight1.Config_kI(slots::Forward, 0.0);
    motorRight1.Config_kD(slots::Forward, 0.04);

    motorRight2.Config_kF(slots::Forward, 0.0);
    motorRight2.Config_kP(slots::Forward, 0.1);
    motorRight2.Config_kI(slots::Forward, 0.0);
    motorRight2.Config_kD(slots::Forward, 0.04);

    motorRight3.Config_kF(slots::Forward, 0.0);
    motorRight3.Config_kP(slots::Forward, 0.1);
    motorRight3.Config_kI(slots::Forward, 0.0);
    motorRight3.Config_kD(slots::Forward, 0.04);

    motorLeft1.ConfigNominalOutputForward(0);
    motorLeft1.ConfigNominalOutputReverse(0);
    motorLeft1.ConfigPeakOutputForward(1);
    motorLeft1.ConfigPeakOutputReverse(-1);

    motorLeft2.ConfigNominalOutputForward(0);
    motorLeft2.ConfigNominalOutputReverse(0);
    motorLeft2.ConfigPeakOutputForward(1);
    motorLeft2.ConfigPeakOutputReverse(-1);

    motorLeft3.ConfigNominalOutputForward(0);
    motorLeft3.ConfigNominalOutputReverse(0);
    motorLeft3.ConfigPeakOutputForward(1);
    motorLeft3.ConfigPeakOutputReverse(-1);

    motorRight1.ConfigNominalOutputForward(0);
    motorRight1.ConfigNominalOutputReverse(0);
    motorRight1.ConfigPeakOutputForward(1);
    motorRight1.ConfigPeakOutputReverse(-1);

    motorRight2.ConfigNominalOutputForward(0);
    motorRight2.ConfigNominalOutputReverse(0);
    motorRight2.ConfigPeakOutputForward(1);
    motorRight2.ConfigPeakOutputReverse(-1);

    motorRight3.ConfigNominalOutputForward(0);
    motorRight3.ConfigNominalOutputReverse(0);
    motorRight3.ConfigPeakOutputForward(1);
    motorRight3.ConfigPeakOutputReverse(-1);

    //Remote Sensor
    motorLeft1.ConfigRemoteFeedbackFilter(0, RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, Remote0);
    motorLeft2.ConfigRemoteFeedbackFilter(0, RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, Remote0);
    motorLeft3.ConfigRemoteFeedbackFilter(0, RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, Remote0);
    motorRight1.ConfigRemoteFeedbackFilter(0, RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, Remote0);
    motorRight2.ConfigRemoteFeedbackFilter(0, RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, Remote0);
    motorRight3.ConfigRemoteFeedbackFilter(0, RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, Remote0);

    motorLeft1.ConfigAuxPIDPolarity(true);
    motorLeft2.ConfigAuxPIDPolarity(true);
    motorLeft3.ConfigAuxPIDPolarity(true);
    motorRight1.ConfigAuxPIDPolarity(true);
    motorRight2.ConfigAuxPIDPolarity(true);
    motorRight3.ConfigAuxPIDPolarity(true);

    motorLeft1.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, PIDind::aux);
    motorLeft2.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, PIDind::aux);
    motorLeft3.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, PIDind::aux);
    motorRight1.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, PIDind::aux);
    motorRight2.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, PIDind::aux);
    motorRight3.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, PIDind::aux);

    motorLeft1.Config_kF(slots::Turning, 0.0);
    motorLeft1.Config_kP(slots::Turning, 0.25); //.25
    motorLeft1.Config_kI(slots::Turning, 0.00);
    motorLeft1.Config_kD(slots::Turning, 0.05); //.02

    motorLeft2.Config_kF(slots::Turning, 0.0);
    motorLeft2.Config_kP(slots::Turning, 0.25); //.25
    motorLeft2.Config_kI(slots::Turning, 0.00);
    motorLeft2.Config_kD(slots::Turning, 0.05); //.02

    motorLeft3.Config_kF(slots::Turning, 0.0);
    motorLeft3.Config_kP(slots::Turning, 0.25); //.25
    motorLeft3.Config_kI(slots::Turning, 0.00);
    motorLeft3.Config_kD(slots::Turning, 0.05); //.02

    motorRight1.Config_kF(slots::Turning, 0.0);
    motorRight1.Config_kP(slots::Turning, 0.25);
    motorRight1.Config_kI(slots::Turning, 0.000);
    motorRight1.Config_kD(slots::Turning, 0.05);

    motorRight2.Config_kF(slots::Turning, 0.0);
    motorRight2.Config_kP(slots::Turning, 0.25);
    motorRight2.Config_kI(slots::Turning, 0.000);
    motorRight2.Config_kD(slots::Turning, 0.05);

    motorRight3.Config_kF(slots::Turning, 0.0);
    motorRight3.Config_kP(slots::Turning, 0.25);
    motorRight3.Config_kI(slots::Turning, 0.000);
    motorRight3.Config_kD(slots::Turning, 0.05);

    motorLeft1.SelectProfileSlot(slots::Forward, PIDind::primary);
    motorLeft1.SelectProfileSlot(slots::Turning, PIDind::aux);
    motorLeft2.SelectProfileSlot(slots::Forward, PIDind::primary);
    motorLeft2.SelectProfileSlot(slots::Turning, PIDind::aux);
    motorLeft3.SelectProfileSlot(slots::Forward, PIDind::primary);
    motorLeft3.SelectProfileSlot(slots::Turning, PIDind::aux);

    motorRight1.SelectProfileSlot(slots::Forward, PIDind::primary);
    motorRight1.SelectProfileSlot(slots::Turning, PIDind::aux);
    motorRight2.SelectProfileSlot(slots::Forward, PIDind::primary);
    motorRight2.SelectProfileSlot(slots::Turning, PIDind::aux);
    motorRight3.SelectProfileSlot(slots::Forward, PIDind::primary);
    motorRight3.SelectProfileSlot(slots::Turning, PIDind::aux);

    motorLeft1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 5);
    motorLeft2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 5);
    motorLeft3.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 5);
    motorRight1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 5);
    motorRight2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 5);
    motorRight3.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 5);

    motorLeft1.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::aux);
    motorLeft1.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::primary);
    motorLeft2.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::aux);
    motorLeft2.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::primary);
    motorLeft3.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::aux);
    motorLeft3.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::primary);
    motorRight1.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::aux);
    motorRight1.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::primary);
    motorRight2.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::aux);
    motorRight2.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::primary);
    motorRight3.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::aux);
    motorRight3.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::primary);

    chooseDriveLimit.SetDefaultOption(sLimited, sLimited);
    chooseDriveLimit.AddOption(sUnlimited, sUnlimited);
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

    // CAN
    motorLeft1.Set(forward - turn);
    motorLeft2.Set(forward - turn);
    motorLeft3.Set(forward - turn);
    motorRight1.Set(forward + turn);
    motorRight2.Set(forward + turn);
    motorRight3.Set(forward + turn);
}

// Stop!
void Drivebase::Stop()
{
    // CAN
    motorLeft1.StopMotor();
    motorLeft2.StopMotor();
    motorLeft3.StopMotor();
    motorRight1.StopMotor();
    motorRight2.StopMotor();
    motorRight3.StopMotor();
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
    motorLeft1.SetSelectedSensorPosition(0);
    motorLeft2.SetSelectedSensorPosition(0);
    motorLeft3.SetSelectedSensorPosition(0);
    motorRight1.SetSelectedSensorPosition(0);
    motorRight2.SetSelectedSensorPosition(0);
    motorRight3.SetSelectedSensorPosition(0);
}

double Drivebase::GetEncoderPositionLeft()
{
    return motorLeft1.GetSelectedSensorPosition(0) * kScaleFactor;
}

double Drivebase::GetEncoderPositionRight()
{
    return motorRight1.GetSelectedSensorPosition(0) * kScaleFactor;
}

double Drivebase::GetEncoderPosition()
{
    return (GetEncoderPositionLeft() + GetEncoderPositionRight()) / 2.0;
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
    return yaw;
}

void Drivebase::DriveForward(double distance, double currentLimit)
{
    motorLeft1.ConfigNominalOutputForward(0);
    motorLeft1.ConfigNominalOutputReverse(0);
    motorLeft1.ConfigPeakOutputForward(currentLimit); // TODO: Looks wrong, verify....
    motorLeft1.ConfigPeakOutputReverse(-currentLimit);

    motorLeft2.ConfigNominalOutputForward(0);
    motorLeft2.ConfigNominalOutputReverse(0);
    motorLeft2.ConfigPeakOutputForward(currentLimit); // TODO: Looks wrong, verify....
    motorLeft2.ConfigPeakOutputReverse(-currentLimit);

    motorLeft3.ConfigNominalOutputForward(0);
    motorLeft3.ConfigNominalOutputReverse(0);
    motorLeft3.ConfigPeakOutputForward(currentLimit); // TODO: Looks wrong, verify....
    motorLeft3.ConfigPeakOutputReverse(-currentLimit);

    motorRight1.ConfigNominalOutputForward(0);
    motorRight1.ConfigNominalOutputReverse(0);
    motorRight1.ConfigPeakOutputForward(currentLimit);
    motorRight1.ConfigPeakOutputReverse(-currentLimit);

    motorRight2.ConfigNominalOutputForward(0);
    motorRight2.ConfigNominalOutputReverse(0);
    motorRight2.ConfigPeakOutputForward(currentLimit);
    motorRight2.ConfigPeakOutputReverse(-currentLimit);

    motorRight3.ConfigNominalOutputForward(0);
    motorRight3.ConfigNominalOutputReverse(0);
    motorRight3.ConfigPeakOutputForward(currentLimit);
    motorRight3.ConfigPeakOutputReverse(-currentLimit);

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

void Drivebase::TurnAbs(double heading)
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

bool Drivebase::TurnRel(double degrees)
{
    target = GetGyroHeading() + degrees;
    double error = target - navx.GetYaw();

    if(error < 0.5)
    {
        return true;
    }

    if (std::abs(error) < 8)
    {
        iAcc += error / 0.02;
    }
    else
    {
        iAcc = 0;
    }

    double dError = (error - prevError_rel) / 0.02; // [Inches/second]
    prevError_rel = error;

    Arcade(0.0, ((error * -KP_ROTATION) + (iAcc * -KI_ROTATION) + (dError * -KD_ROTATION)));
    return false;
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
    sensorOverride = active;

    ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration lim;
    lim.currentLimit = 60;
    lim.triggerThresholdCurrent = 60;
    lim.triggerThresholdTime = 0.2;
    lim.enable = !sensorOverride;
    motorLeft1.ConfigSupplyCurrentLimit(lim);
    motorLeft2.ConfigSupplyCurrentLimit(lim);
    motorLeft3.ConfigSupplyCurrentLimit(lim);
    motorRight1.ConfigSupplyCurrentLimit(lim);
    motorRight2.ConfigSupplyCurrentLimit(lim);
    motorRight3.ConfigSupplyCurrentLimit(lim);

    if (sensorOverride)
    {
        motorRight1.ConfigOpenloopRamp(0.0);
        motorRight2.ConfigOpenloopRamp(0.0);
        motorRight3.ConfigOpenloopRamp(0.0);
        motorLeft1.ConfigOpenloopRamp(0.0);
        motorLeft2.ConfigOpenloopRamp(0.0);
        motorLeft3.ConfigOpenloopRamp(0.0);
    }
    else
    {
        motorRight1.ConfigOpenloopRamp(0.2);
        motorRight2.ConfigOpenloopRamp(0.2);
        motorRight3.ConfigOpenloopRamp(0.2);
        motorLeft1.ConfigOpenloopRamp(0.2);
         motorLeft2.ConfigOpenloopRamp(0.2);
          motorLeft3.ConfigOpenloopRamp(0.2);
    }

    // TODO: Check if there are more settings that should be changed.
}

// SmartDash updater
void Drivebase::UpdateSmartdash()
{
    SmartDashboard::PutNumber("Drive L1", motorLeft1.Get());
    SmartDashboard::PutNumber("Drive L2", motorLeft2.Get());
    SmartDashboard::PutNumber("Drive L3", motorLeft3.Get());
    SmartDashboard::PutNumber("Drive R1", motorRight1.Get());
    SmartDashboard::PutNumber("Drive R2", motorRight2.Get());
    SmartDashboard::PutNumber("Drive R3", motorRight3.Get());

    SmartDashboard::PutBoolean("DriveShifter", solenoidShifter.Get());

    SmartDashboard::PutNumber("DriveEncL", GetEncoderPositionLeft());
    SmartDashboard::PutNumber("DriveEncR", GetEncoderPositionRight());

    SmartDashboard::PutBoolean("DriveOverride", sensorOverride);

    SmartDashboard::PutNumber("Gyro Heading", GetGyroHeading());

    SmartDashboard::PutNumber("Heading Setpoint", forwardHeading);

    SmartDashboard::PutData("_DriveLimits", &chooseDriveLimit);
    SensorOverride(chooseDriveLimit.GetSelected() == sLimited);
}
