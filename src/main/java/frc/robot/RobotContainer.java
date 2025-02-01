// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {

  private final Swerve swerve;
  private final Elevator elevator;

  private final EndEffector endEffector;

  SendableChooser<Command> chooser = new SendableChooser<>();

  @NotLogged private final CommandXboxController xboxController;
  private boolean colour;

  public RobotContainer() {
    swerve = new Swerve();
    elevator = new Elevator();
    endEffector = new EndEffector();

    xboxController = new CommandXboxController(0);

    chooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    defualtCommands();
  }

  private void configureBindings() {

    Trigger testEffector = xboxController.axisGreaterThan(0, 0.05);
    testEffector.onTrue(endEffector.setVelocityCommand(EndEffectorConstants.INTAKE_SPEED));
    testEffector.onFalse(endEffector.setVelocityCommand(AngularVelocity.ofBaseUnits(0, Units.RPM)));

    Trigger intakeEffector = xboxController.a();
    intakeEffector.onTrue(endEffector.intakeCommand());
    intakeEffector.onFalse(
        endEffector.setVelocityCommand(AngularVelocity.ofBaseUnits(0, Units.RPM)));

    Trigger outtakeEffector = xboxController.b();
    outtakeEffector.onTrue(endEffector.intakeCommand());
    outtakeEffector.onFalse(
        endEffector.setVelocityCommand(AngularVelocity.ofBaseUnits(0, Units.RPM)));

    Trigger zero = xboxController.y();

    zero.onTrue(swerve.zeroRobotCommad(colour));
  }

  private void defualtCommands() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            xboxController::getLeftY, xboxController::getLeftX, xboxController::getRightX));
  }

  public void teleopInit() {
    colour = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.getColour(colour);
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
