// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralEE.h"

#include <frc2/command/Commands.h>

using namespace ctre::phoenix::motorcontrol;

namespace Subsystems
{

  CoralEE::CoralEE() : m_motorController{62}
  {
    m_motorController.SetInverted(false);
    m_motorController.SetNeutralMode(NeutralMode::Brake);
  }

  void CoralEE::Periodic() {}

  void CoralEE::SetPower(double power)
  {
    m_motorController.Set(power * 1.0);
  }

  frc2::CommandPtr CoralEE::Eject_Command()
  {
    return frc2::cmd::Sequence(
        RunOnce([this]()
                { SetPower(1.0); }),
        frc2::cmd::Wait(1.5_s), // TODO probably add photogate
        RunOnce([this]()
                { SetPower(0.0); }));
  }

  frc2::CommandPtr CoralEE::EjectL4_Command()
  {
    return frc2::cmd::Sequence(
        RunOnce([this]()
                { SetPower(-1.0); }),
        frc2::cmd::Wait(1.5_s), // TODO probably add photogate
        RunOnce([this]()
                { SetPower(0.0); }));
  }

}
