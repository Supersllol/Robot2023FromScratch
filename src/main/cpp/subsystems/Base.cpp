#include "subsystems/Base.h"
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



Base::Base() {
  // Implementation of subsystem constructor goes here.
    m_leftMotorLead.RestoreFactoryDefaults();
    m_leftMotorLead.SetInverted(false);
    
    m_LeftEncoder.SetPositionConversionFactor(DriveConstant::kWheelCirconfM / DriveConstant::kGearBoxRatio);
    m_LeftEncoder.SetVelocityConversionFactor(DriveConstant::kWheelCirconfM / DriveConstant::kGearBoxRatio / 60); // meter/s
    
    m_LeftPidController.SetFeedbackDevice(m_LeftEncoder);
    m_LeftPidController.SetP(DriveConstant::kPDriveVelocity);

    m_leftMotorFollow.Follow(m_leftMotorLead);
    m_leftMotorFollow.SetInverted(false);
 
    m_rightMotorLead.RestoreFactoryDefaults();
    m_rightMotorLead.SetInverted(false);

    m_RightEncoder.SetPositionConversionFactor(DriveConstant::kWheelCirconfM / DriveConstant::kGearBoxRatio); // motor native to meters
    m_RightEncoder.SetVelocityConversionFactor(DriveConstant::kWheelCirconfM / DriveConstant::kGearBoxRatio / 60); // meter/s

    m_RightPidController.SetFeedbackDevice(m_RightEncoder);
    m_RightPidController.SetP(DriveConstant::kPDriveVelocity);
    
    m_rightMotorFollow.Follow(m_rightMotorLead);
    m_rightMotorFollow.SetInverted(false);

    m_RobotDrive.reset(new frc::DifferentialDrive(m_leftMotorLead, m_rightMotorLead));
    m_pOdometry.reset(new frc::DifferentialDriveOdometry(units::degree_t{m_Gyro.GetYaw()}, units::meter_t(m_LeftEncoder.GetPosition()),
    units::meter_t(m_RightEncoder.GetPosition())));
    
}

void Base::Periodic() {
    m_pOdometry->Update(m_Gyro.GetRotation2d(), units::meter_t{m_LeftEncoder.GetVelocity()}, units::meter_t{m_RightEncoder.GetVelocity()});
}

void Base::ArcadeDrive(double moveValue, double rotateValue)
{
  m_RobotDrive->ArcadeDrive(moveValue, rotateValue);
}

void Base::ResetEncoders()
{
  m_LeftEncoder.SetPosition(0);
  m_RightEncoder.SetPosition(0);
  
}

frc::DifferentialDriveWheelSpeeds Base::GetWheelSpeeds() {
  return {units::meters_per_second_t{m_LeftEncoder.GetVelocity()}, units::meters_per_second_t{m_RightEncoder.GetVelocity()}};
}

units::degree_t Base::GetHeading() {
  return units::degree_t{m_Gyro.GetYaw()};
}

frc::Pose2d Base::GetPose(){
  return m_pOdometry->GetPose();
}

void Base::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotorLead.SetVoltage(left);
  m_rightMotorLead.SetVoltage(right);
  m_RobotDrive->Feed();
}

void Base::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_pOdometry->ResetPosition(m_Gyro.GetRotation2d(),
                           units::meter_t{m_LeftEncoder.GetVelocity()},
                           units::meter_t{m_RightEncoder.GetVelocity()}, pose);
}

void Base::ResetGyro(){
  m_Gyro.Reset();
}