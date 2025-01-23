// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {

  private final Swerve swerve;
  private final Elevator elevator;

  private final EndEffector endEffector;

  @NotLogged private final CommandXboxController xboxController;

  public RobotContainer() {
    swerve = new Swerve();
    elevator = new Elevator();
    endEffector = new EndEffector();

    xboxController = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            xboxController::getLeftY, xboxController::getLeftX, xboxController::getRightX));

    Trigger intakeEffector = xboxController.axisGreaterThan(0, 0.05);
    intakeEffector.onTrue(endEffector.setVelocityCommand(EndEffectorConstants.INTAKE_SPEED));
    intakeEffector.onFalse(endEffector.setVelocityCommand(AngularVelocity.ofBaseUnits(0, Units.RPM)));
  }

  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
