// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"
#include <frc/drive/DifferentialDrive.h>
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include <AHRS.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
class Base : public frc2::SubsystemBase {
 public:
  Base();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr ExampleMethodCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool ExampleCondition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void ArcadeDrive(double, double);
  void ResetEncoders();
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
  units::degree_t GetHeading();
  frc::Pose2d GetPose();
  void TankDriveVolts(units::volt_t, units::volt_t);
  void ResetOdometry(frc::Pose2d);
  void ResetGyro();
private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax 	m_leftMotorLead {DriveConstant::DriveGaucheMaitreID, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax 	m_leftMotorFollow {DriveConstant::DriveGaucheEsclaveID, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax 	m_rightMotorLead {DriveConstant::DriveDroitMaitreID, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax 	m_rightMotorFollow {DriveConstant::DriveDroitEsclaveID, rev::CANSparkMax::MotorType::kBrushless};
	std::shared_ptr<frc::DifferentialDrive> 	m_RobotDrive;
	rev::SparkMaxRelativeEncoder 		m_LeftEncoder = m_leftMotorLead.rev::CANSparkMax::GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, DriveConstant::kWheelPulsesPerTurn);
	rev::SparkMaxRelativeEncoder 		m_RightEncoder = m_rightMotorLead.rev::CANSparkMax::GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, DriveConstant::kWheelPulsesPerTurn);
	rev::SparkMaxPIDController	m_LeftPidController = m_leftMotorLead.GetPIDController();
	rev::SparkMaxPIDController	m_RightPidController =  m_rightMotorLead.GetPIDController();
  std::shared_ptr<frc::DifferentialDriveOdometry> m_pOdometry;
  AHRS m_Gyro{frc::SerialPort::Port::kUSB};
};
