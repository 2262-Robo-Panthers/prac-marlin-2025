// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include "commands/Autos.h"
#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_subsystemDrive.SetDefaultCommand(
    frc2::RunCommand(
      [this]() {
        m_subsystemDrive.SetMotion(
          -frc::ApplyDeadband(m_driveTrainController.GetRightY(), 0.07),
          -frc::ApplyDeadband(m_driveTrainController.GetRightX(), 0.07), // negative: robot X-axis points left
          -frc::ApplyDeadband(m_driveTrainController.GetLeftX(), 0.07), // negative: positive angles go left
          Types::CoordinateSystem::FIELD
        );
      },
      {&m_subsystemDrive}
    )
  );

  m_driveTrainController.A().OnTrue(
    new frc2::InstantCommand(
      [this]() {
        m_subsystemDrive.ResetOdometryRotation(0_deg);
      },
      {&m_subsystemDrive}
    )
  );

  m_driveTrainController.B().OnTrue(
    new frc2::InstantCommand(
      [this]() {
        m_subsystemDrive.ResetOdometryRotation(180_deg);
      },
      {&m_subsystemDrive}
    )
  );

  m_driveTrainController.X().OnTrue(
    new frc2::InstantCommand(
      [this]() {
        m_subsystemDrive.ResetOdometryTranslation({0_m, 0_m});
      },
      {&m_subsystemDrive}
    )
  );

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto();
}
