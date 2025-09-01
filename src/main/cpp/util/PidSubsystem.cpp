// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <units/time.h>

#include "util/PidSubsystem.h"

using namespace rev::spark;

PIDSubsystem::PIDSubsystem(Types::CanId canId, Types::PidCoefficients pid) :
  m_motorController{canId, SparkMax::MotorType::kBrushless},
  m_encoder{m_motorController.GetEncoder()},
  m_pidLoop{m_motorController.GetClosedLoopController()},
  m_pid{pid}
{
}

void PIDSubsystem::Initialize() {
  m_motorController.Configure(
    m_motorConfig,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );

  m_encoder.SetPosition(0.0),
  m_pidLoop.SetReference(0.0, SparkLowLevel::ControlType::kPosition);
}

// This method will be called once per scheduler run
void PIDSubsystem::Periodic() {
  if (!m_isNeutralized) {
    m_pidLoop.SetReference(m_desiredPosition, SparkLowLevel::ControlType::kPosition);
  }
}

double PIDSubsystem::GetPosition() const {
  return m_encoder.GetPosition();
}

void PIDSubsystem::ResetPosition(double position) {
  m_desiredPosition = position;
  m_encoder.SetPosition(position);
  m_pidLoop.SetIAccum(0.0);
  m_pidLoop.SetReference(position, SparkLowLevel::ControlType::kPosition);
}

void PIDSubsystem::MovePosition(double change, double min, double max) {
  GoToPosition(m_desiredPosition + change, min, max);
}

void PIDSubsystem::GoToPosition(double position, double min, double max) {
  m_desiredPosition = std::clamp(position, min, max);
}

bool PIDSubsystem::IsInPosition(double position, double tolerance) const {
  return tolerance < 0.0 || std::abs(GetPosition() - position) <= tolerance;
}

frc2::CommandPtr PIDSubsystem::GoToPosition_Command(double position) {
  return
    RunOnce(
      [this, position]() {
        GoToPosition(position);
      }
    )

    .AndThen(
    frc2::cmd::Race(

      frc2::cmd::WaitUntil(
        [this, position]() {
          return IsInPosition(position);
        }
      ),
      frc2::cmd::Wait(2.0_s)
        .AndThen(frc2::cmd::RunOnce(
          [this]() {
            ResetPosition(m_encoder.GetPosition());
            FRC_ReportWarning("Movement command timed out.");
          }
        ))

    ));
}

frc2::CommandPtr PIDSubsystem::GoToPositionBlind_Command(double position) {
  return RunOnce(
    [this, position]() {
      GoToPosition(position);
    }
  );
}

void PIDSubsystem::ToggleKillSwitch() {
  ToggleKillSwitch(!m_isNeutralized);
}

void PIDSubsystem::ToggleKillSwitch(bool value) {
  if (value) {
    m_isNeutralized = true;
  
    m_motorConfig.closedLoop
      .Pid(0.0, 0.0, 0.0);
  }
  else {
    m_isNeutralized = false;

    m_motorConfig.closedLoop
      .Pid(m_pid.p, m_pid.i, m_pid.d);

    ResetPosition(m_encoder.GetPosition());
  }

  m_motorController.Configure(
    m_motorConfig,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );
}
