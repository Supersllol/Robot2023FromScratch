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
    //frc::SmartDashboard::PutNumber("EncoderDistance", m_pBase->m_LeftEncoder.GetPosition());
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