// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/SafetIntakeL4Command.h"

SafetIntakeL4Command::SafetIntakeL4Command() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SafetIntakeL4Command::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SafetIntakeL4Command::Execute() {}

// Called once the command ends or is interrupted.
void SafetIntakeL4Command::End(bool interrupted) {}

// Returns true when the command should end.
bool SafetIntakeL4Command::IsFinished() {
  return false;
}
