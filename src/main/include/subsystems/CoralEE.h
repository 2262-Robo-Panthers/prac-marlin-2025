// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

class CoralEE : public frc2::SubsystemBase {
 public:
  CoralEE();

  void Periodic() override;

  /**
   * Requests the motor to spin at some fraction of a scaled full power.
   *
   * @param power The requested power in [-1, 1], before applying the coefficient.
   */
  void SetPower(double power);

  /**
   * Runs the motor for long enough to probably eject the coral.
   */
  frc2::CommandPtr Eject_Command();
  frc2::CommandPtr EjectL4_Command();

private:
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_motorController;
};

