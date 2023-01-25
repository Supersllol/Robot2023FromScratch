// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandJoystick.h>
#include "Constants.h"
#include "subsystems/Base.h"
#include <frc2/command/button/Trigger.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
#include "commands/AutonomousCommand.h"
#include "commands/ManualDrive.h"
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();
  frc2::CommandJoystick m_ThrottleStick{OIConstant::ThrottleStickID};
  frc2::CommandJoystick m_TurnStick{OIConstant::TurnStickID};
  frc2::CommandXboxController m_CoPilotController{OIConstant::CoPilotControllerID};

  frc2::Command* GetAutonomousCommand();
  double GetAngle();
  void ResetGyro();
  void SetRightMotorsTeleop();
  void SetRightMotorsAutonomous();
 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  
  // The robot's subsystems are defined here...
  Base m_Base;

  void ConfigureBindings();
};
