// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/SafeL4Commands.h"

using namespace Subsystems;

frc2::CommandPtr SafeScore_Command(Elevator *elevator, L4CoralArm *l4CoralArm, double height) {
  return l4CoralArm->GTFO_Command()
    .AndThen(elevator->GoToPosition_Command(height))
    .AndThen(l4CoralArm->GoToPosition_Command(0.0));
}

frc2::CommandPtr SafeScoreL4_Command(Elevator *elevator, L4CoralArm *l4CoralArm) {
  return l4CoralArm->GTFO_Command()
    .AndThen(elevator->GoToPosition_Command(1.0))
    .AndThen(l4CoralArm->GoToPosition_Command(0.82));
}

frc2::CommandPtr SafeIntake_Command(Elevator *elevator, L4CoralArm *l4CoralArm) {
  return l4CoralArm->GTFO_Command()
    .AndThen(elevator->GoToPosition_Command(0.18))
    .AndThen(l4CoralArm->GoToPosition_Command(-0.07));
}
