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

#define AXIS(ctrl, bind) frc::ApplyDeadband(ctrl.bind(), 0.07)
#define Y_AXIS(ctrl, bind) (-AXIS(ctrl, bind))

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

  /**
  *** Drive
  **/

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

  /**
  *** Elevator
  **/

  m_subsystemElevator.SetDefaultCommand(
    frc2::RunCommand(
      [this]() {
        m_subsystemElevator.MovePosition(
          frc::ApplyDeadband(
            m_endEffectorController.GetRightTriggerAxis() -
            m_endEffectorController.GetLeftTriggerAxis(), 0.07
          ) * 0.01
        );
      },
      {&m_subsystemElevator}
    )
  );

  m_endEffectorController.A().OnTrue(
    m_subsystemElevator.GoToPosition_Command(0.0)
  );

  m_endEffectorController.Y().OnTrue(
    m_subsystemElevator.GoToPosition_Command(1.0)
  );

  m_endEffectorController.X().OnTrue(
    m_subsystemElevator.GoToPosition_Command(0.18)
  );

  m_endEffectorController.POVDown().OnTrue(
    m_subsystemElevator.GoToPosition_Command(0.35)
  );

  m_endEffectorController.POVLeft().OnTrue(
    m_subsystemElevator.GoToPosition_Command(0.54)
  );

  m_endEffectorController.POVRight().OnTrue(
    m_subsystemElevator.GoToPosition_Command(0.95)
  );

  m_endEffectorController.POVUp().OnTrue(
    m_subsystemElevator.GoToPosition_Command(1.0)
  );

  ( m_endEffectorController.LeftBumper() &&
    m_endEffectorController.Back() ).OnTrue(
    new frc2::InstantCommand(
      [this]() {
        m_subsystemElevator.ResetPosition(0.0);
      },
      {&m_subsystemElevator}
    )
  );

  ( m_endEffectorController.RightBumper() &&
    m_endEffectorController.Start() ).OnTrue(
    new frc2::InstantCommand(
      [this]() {
        m_subsystemElevator.ResetPosition(1.0);
      },
      {&m_subsystemElevator}
    )
  );

  ( m_endEffectorController.LeftBumper() &&
    m_endEffectorController.RightBumper() ).OnTrue(
    new frc2::InstantCommand(
      [this]() {
        m_subsystemElevator.ToggleKillSwitch();
      },
      {&m_subsystemElevator}
    )
  );

  /**
  *** CoralEE
  **/

  m_subsystemCoralEE.SetDefaultCommand(
    frc2::RunCommand(
      [this]() {
        m_subsystemCoralEE.SetPower(
          Y_AXIS(m_endEffectorController, GetLeftY)
        );
      },
      {&m_subsystemCoralEE}
    )
  );

  /**
  *** L4CoralArm
  **/

  m_subsystemL4CoralArm.SetDefaultCommand(
    frc2::RunCommand{
      [this]() {
        m_subsystemL4CoralArm.MovePosition(
          Y_AXIS(m_endEffectorController, GetRightY) * 0.01
        );
      },
      {&m_subsystemL4CoralArm}
    }
  );

}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto();
}
