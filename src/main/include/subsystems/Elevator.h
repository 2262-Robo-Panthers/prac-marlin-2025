// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "util/PidSubsystem.h"
#include "Types.h"

namespace Subsystems {

class Elevator : public PidSubsystem {
public:
  Elevator();

private:
  rev::spark::SparkMax m_motorFollower;
  rev::spark::SparkMaxConfig m_followerConfig;
};

}
