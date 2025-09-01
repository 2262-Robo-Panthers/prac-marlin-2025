// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/L4CoralArm.h"
#include "Types.h"

using namespace rev::spark;

namespace Subsystems {

L4CoralArm::L4CoralArm() :
  PIDSubsystem{60, Types::PidCoefficients{1.4, 0.0, 0.5}}
{
  m_motorConfig
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(20);
  m_motorConfig.encoder
    .PositionConversionFactor(1.0 / 131.1);
  m_motorConfig.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    .Pid(1.4, 0.0, 0.5)
    .OutputRange(-1, 1);

  Initialize();
}

}
