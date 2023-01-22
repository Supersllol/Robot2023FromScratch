#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Base.h"

class ManualDrive
    : public frc2::CommandHelper<frc2::CommandBase, ManualDrive> {
private:
    Base* m_Base;
    std::function<double()> m_GetY;
    std::function<double()> m_GetX;
    
public:
    explicit ManualDrive(Base* p_base, std::function<double()> GetY, std::function<double()> GetX);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool) override;
};