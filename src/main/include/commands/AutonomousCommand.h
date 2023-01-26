#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Base.h"
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Have the robot drive tank style using the PS3 Joystick until interrupted.
 */
class AutonomousCommand : public frc2::CommandHelper<frc2::CommandBase, AutonomousCommand> {
 public:
  AutonomousCommand(Base* drivetrain);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  Base* m_pBase;
};