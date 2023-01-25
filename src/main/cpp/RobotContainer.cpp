// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Configure the button bindings
  ConfigureBindings();
  m_Base.SetDefaultCommand(ManualDrive(&m_Base,
    [this] { return m_TurnStick.GetX(); },
    [this] { return m_ThrottleStick.GetY(); }));
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  //frc2::Trigger([this] {
  //  return m_Base.ExampleCondition();
  //}).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  //m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{frc::SimpleMotorFeedforward<units::meters>
  {DriveConstant::ks, DriveConstant::kv, DriveConstant::ka}, DriveConstant::kDriveKinematics, 10_V};

  frc::TrajectoryConfig config{DriveConstant::kMaxSpeed, DriveConstant::kMaxAcceleration};
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(DriveConstant::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    // Start at the origin facing the +X direction
    frc::Pose2d{0_m, 0_m, 0_deg},
    // No intermediate waypoints
    {},
    // End 3 meters straight ahead of where we started, facing forward
    frc::Pose2d{3_m, 0_m, 0_deg},
    // Pass the config
    config);
  // Reset odometry to the starting pose of the trajectory.
  m_Base.ResetOdometry(exampleTrajectory.InitialPose());

  frc2::RamseteCommand ramseteCommand{
      exampleTrajectory,
      [this]() { return m_Base.GetPose(); },
      frc::RamseteController{DriveConstant::kRamseteB,
                             DriveConstant::kRamseteZeta},
      frc::SimpleMotorFeedforward<units::meters>{
          DriveConstant::ks, DriveConstant::kv, DriveConstant::ka},
      DriveConstant::kDriveKinematics,
      [this] { return m_Base.GetWheelSpeeds(); },
      frc2::PIDController{DriveConstant::kPDriveVelocity, 0, 0},
      frc2::PIDController{DriveConstant::kPDriveVelocity, 0, 0},
      [this](auto left, auto right) { m_Base.TankDriveVolts(left, right); },
      {&m_Base}};
    return new frc2::SequentialCommandGroup(
    std::move(ramseteCommand),
    frc2::InstantCommand([this] { m_Base.TankDriveVolts(0_V, 0_V); }, {}));
}

double RobotContainer::GetAngle(){
  return m_Base.GetAngle();
}

void RobotContainer::ResetGyro(){
  m_Base.ResetGyro();
}

void RobotContainer::SetRightMotorsAutonomous(){
  m_Base.SetRightMotorsAutonomous();
}

void RobotContainer::SetRightMotorsTeleop(){
  m_Base.SetRightMotorsTeleop();
}