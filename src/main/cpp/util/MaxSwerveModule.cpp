// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/MaxSwerveModule.h"

#include "Types.h"

using namespace Types::Swerve;
using namespace rev::spark;

MaxSwerveModule::MaxSwerveModule(
  const Types::CanId canIdDrive,
  const Types::CanId canIdSteer,
  const units::radian_t angle
) :
  m_motorControllers{
    .drive = SparkMax{canIdDrive, SparkMax::MotorType::kBrushless},
    .steer = SparkMax{canIdSteer, SparkMax::MotorType::kBrushless},
  },
  m_encoders{
    .drive = m_motorControllers.drive.GetEncoder(),
    .steer = m_motorControllers.steer.GetAbsoluteEncoder(),
  },
  m_pidLoops{
    .drive = m_motorControllers.drive.GetClosedLoopController(),
    .steer = m_motorControllers.steer.GetClosedLoopController(),
  },
  m_chassisAngularOffset{angle}
{
  constexpr double driveFactor = 0.23938936 / 5.5;
  constexpr double driveVelocityFf = 60.0 / 5676 / driveFactor;

  m_configs.drive
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(50);
  m_configs.drive.encoder
    .PositionConversionFactor(driveFactor)
    .VelocityConversionFactor(driveFactor / 60.0);
  m_configs.drive.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.04, 0.0, 0.0)
    .VelocityFF(driveVelocityFf)
    .OutputRange(-1, 1);

  constexpr double steerFactor = 6.28318531;

  m_configs.steer
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(20);
  m_configs.steer.absoluteEncoder
    .Inverted(true)
    .PositionConversionFactor(steerFactor)
    .VelocityConversionFactor(steerFactor / 60.0);
  m_configs.steer.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
    .Pid(1.0, 0.0, 0.0)
    .OutputRange(-1, 1)
    .PositionWrappingEnabled(true)
    .PositionWrappingInputRange(0, steerFactor);

  m_motorControllers.drive.Configure(
    m_configs.drive,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );
  m_motorControllers.steer.Configure(
    m_configs.steer,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );

  m_desiredState.angle = units::radian_t{m_encoders.steer.GetPosition()};

  ResetDriveEncoder();
}

frc::SwerveModulePosition MaxSwerveModule::GetPosition() const {
  return {
    .distance = units::meter_t{m_encoders.drive.GetPosition()},
    .angle = units::radian_t{m_encoders.steer.GetPosition()} + m_chassisAngularOffset,
  };
}

frc::SwerveModuleState MaxSwerveModule::GetState() const {
  return {
    .speed = units::meters_per_second_t{m_encoders.drive.GetVelocity()},
    .angle = units::radian_t{m_encoders.steer.GetPosition()} + m_chassisAngularOffset,
  };
}

void MaxSwerveModule::SetState(const frc::SwerveModuleState &desired) {
  frc::SwerveModuleState offsetted{};
  offsetted.speed = desired.speed;
  offsetted.angle = desired.angle - frc::Rotation2d(m_chassisAngularOffset);
  offsetted.Optimize(units::radian_t{m_encoders.steer.GetPosition()});

  m_pidLoops.drive.SetReference(offsetted.speed.value(), SparkMax::ControlType::kVelocity);
  m_pidLoops.steer.SetReference(offsetted.angle.Radians().value(), SparkMax::ControlType::kPosition);

  m_desiredState = desired;
}

void MaxSwerveModule::ResetDriveEncoder() {
  m_encoders.drive.SetPosition(0);
}
