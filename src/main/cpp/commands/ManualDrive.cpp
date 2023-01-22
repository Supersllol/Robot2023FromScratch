#include "commands/ManualDrive.h"

ManualDrive::ManualDrive(Base* p_base, 
                           std::function<double()> GetY,
                           std::function<double()> GetX)
    : m_Base{p_base}, m_GetY{GetY}, m_GetX{GetX} {
  // Register that this command requires the subsystem.
  AddRequirements(p_base);
}
void ManualDrive::Initialize() {}

void ManualDrive::Execute() {
    double Multi = 1.0;
    m_Base->ArcadeDrive(m_GetY() * 0.8 * Multi, 0.75 * m_GetX() * Multi);
}

bool ManualDrive::IsFinished() {return false;}

void ManualDrive::End(bool) {m_Base->ArcadeDrive(0.0, 0.0);}