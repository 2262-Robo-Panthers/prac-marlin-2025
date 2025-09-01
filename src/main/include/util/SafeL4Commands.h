// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Commands.h>

#include "subsystems/Elevator.h"
#include "subsystems/L4CoralArm.h"

frc2::CommandPtr SafeScore_Command(Subsystems::Elevator *elevator, Subsystems::L4CoralArm *l4CoralArm, double height);
frc2::CommandPtr SafeScoreL4_Command(Subsystems::Elevator *elevator, Subsystems::L4CoralArm *l4CoralArm);
frc2::CommandPtr SafeIntake_Command(Subsystems::Elevator *elevator, Subsystems::L4CoralArm *l4CoralArm);
