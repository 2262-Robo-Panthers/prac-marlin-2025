// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <units/time.h>

#include "subsystems/Elevator.h"

using namespace rev::spark;

namespace Subsystems {

Elevator::Elevator() :
  PIDSubsystem{11, {.p = 1.8, .i = 0.0002, .d = 3.0}},
  m_motorFollower{12, SparkMax::MotorType::kBrushless}
{
  m_motorConfig
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(20);
  m_motorConfig.encoder
    .PositionConversionFactor(1.0 / 59.9);
  m_motorConfig.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    .Pid(1.8, 0.0002, 3.0)
    .OutputRange(-1, 1);

  Initialize();

  m_followerConfig
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(20)
    .Follow(m_motorController, true);

  m_motorFollower.Configure(
    m_followerConfig,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );
}

}
