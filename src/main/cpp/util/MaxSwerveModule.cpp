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
  m_motorControllerDrive{canIdDrive, SparkMax::MotorType::kBrushless},
  m_motorControllerSteer{canIdSteer, SparkMax::MotorType::kBrushless},
  m_encoderDrive{m_motorControllerDrive.GetEncoder()},
  m_encoderSteer{m_motorControllerSteer.GetAbsoluteEncoder()},
  m_pidLoopDrive{m_motorControllerDrive.GetClosedLoopController()},
  m_pidLoopSteer{m_motorControllerSteer.GetClosedLoopController()},
  m_chassisAngularOffset{angle}
{
  constexpr double driveFactor = 0.23938936 / 5.5;
  constexpr double driveVelocityFf = 60.0 / 5676 / driveFactor;

  m_driveConfig
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(50);
  m_driveConfig.encoder
    .PositionConversionFactor(driveFactor)
    .VelocityConversionFactor(driveFactor / 60.0);
  m_driveConfig.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.04, 0.0, 0.0)
    .VelocityFF(driveVelocityFf)
    .OutputRange(-1, 1);

  constexpr double steerFactor = 6.28318531;

  m_steerConfig
    .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
    .SmartCurrentLimit(20);
  m_steerConfig.absoluteEncoder
    .Inverted(true)
    .PositionConversionFactor(steerFactor)
    .VelocityConversionFactor(steerFactor / 60.0);
  m_steerConfig.closedLoop
    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
    .Pid(1.0, 0.0, 0.0)
    .OutputRange(-1, 1)
    .PositionWrappingEnabled(true)
    .PositionWrappingInputRange(0, steerFactor);

  m_motorControllerDrive.Configure(
    m_driveConfig,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );
  m_motorControllerSteer.Configure(
    m_steerConfig,
    SparkBase::ResetMode::kResetSafeParameters,
    SparkBase::PersistMode::kPersistParameters
  );

  m_desiredState.angle = units::radian_t{m_encoderSteer.GetPosition()};

  ResetDriveEncoder();
}

frc::SwerveModulePosition MaxSwerveModule::GetPosition() const {
  return {
    .distance = units::meter_t{m_encoderDrive.GetPosition()},
    .angle = units::radian_t{m_encoderSteer.GetPosition()} + m_chassisAngularOffset,
  };
}

frc::SwerveModuleState MaxSwerveModule::GetState() const {
  return {
    .speed = units::meters_per_second_t{m_encoderDrive.GetVelocity()},
    .angle = units::radian_t{m_encoderSteer.GetPosition()} + m_chassisAngularOffset,
  };
}

void MaxSwerveModule::SetState(const frc::SwerveModuleState &desired) {
  frc::SwerveModuleState offsetted{};
  offsetted.speed = desired.speed;
  offsetted.angle = desired.angle - frc::Rotation2d(m_chassisAngularOffset);
  offsetted.Optimize(units::radian_t{m_encoderSteer.GetPosition()});

  m_pidLoopDrive.SetReference(offsetted.speed.value(), SparkMax::ControlType::kVelocity);
  m_pidLoopSteer.SetReference(offsetted.angle.Radians().value(), SparkMax::ControlType::kPosition);

  m_desiredState = desired;
}

void MaxSwerveModule::ResetDriveEncoder() {
  m_encoderDrive.SetPosition(0);
}
