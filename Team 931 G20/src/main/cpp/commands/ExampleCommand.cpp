// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExampleCommand.h"

RunIntake::RunIntake(Intake& subsystem)
    : m_subsystem{subsystem} {
        AddRequirements(& subsystem);
    }

void RunIntake::Initialize() {
    m_subsystem.SetDeployed(true);
}

void RunIntake::End(bool) {
    m_subsystem.SetDeployed(false);
}
