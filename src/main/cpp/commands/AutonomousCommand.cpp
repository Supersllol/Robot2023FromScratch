#include "commands/AutonomousCommand.h"

AutonomousCommand::AutonomousCommand(Base* p_base)
    : m_pBase(p_base) {
  AddRequirements(m_pBase);
}

void AutonomousCommand::Initialize() {
    m_pBase->ResetGyro();
}
// Called repeatedly when this Command is scheduled to run
void AutonomousCommand::Execute() 
{ 
    frc::SmartDashboard::PutNumber("RightEncoderDistance", (m_pBase->GetRightEncoder()).GetPosition() * (m_pBase->GetRightEncoder()).GetPositionConversionFactor());
    frc::SmartDashboard::PutNumber("LeftEncoderDistance", (m_pBase->GetLeftEncoder()).GetPosition() * (m_pBase->GetLeftEncoder()).GetPositionConversionFactor());
}

// Make this return true when this Command no longer needs to run execute()
bool AutonomousCommand::IsFinished() 
{ 
    return false; 
}

// Called once after isFinished returns true
void AutonomousCommand::End(bool) 
{ 
    m_pBase->ArcadeDrive(0, 0); 
}