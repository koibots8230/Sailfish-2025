// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;


@Logged
public class RobotContainer {

  private final Swerve swerve;
  private final Elevator elevator;

  private final EndEffector endEffector;

  @NotLogged private final CommandXboxController xboxController;
  private final XboxController controller;
  private boolean colour;

  public RobotContainer() {
    swerve = new Swerve();
    elevator = new Elevator();
    endEffector = new EndEffector();
    controller = new XboxController(0);

    xboxController = new CommandXboxController(0);

    configureBindings();
    defualtCommands();
  }

  private void configureBindings() {

    Trigger testEffector = xboxController.axisGreaterThan(0, 0.05);
    testEffector.onTrue(endEffector.setVelocityCommand(EndEffectorConstants.INTAKE_SPEED));
    testEffector.onFalse(endEffector.setVelocityCommand(AngularVelocity.ofBaseUnits(0, Units.RPM)));

    Trigger intakeEffector = xboxController.a();
    intakeEffector.onTrue(endEffector.intakeCommand());
    intakeEffector.onFalse(endEffector.setVelocityCommand(AngularVelocity.ofBaseUnits(0, Units.RPM)));

    Trigger outtakeEffector = xboxController.b();
    outtakeEffector.onTrue(endEffector.intakeCommand());
    outtakeEffector.onFalse(endEffector.setVelocityCommand(AngularVelocity.ofBaseUnits(0, Units.RPM)));

    Trigger zero = new Trigger(() -> controller.getAButton());

    zero.onTrue(swerve.zeroRobotCommad(colour));
  }

  private void defualtCommands(){
    if (colour){
    swerve.setDefaultCommand(swerve.driveFieldRelativeBlueCommand(controller::getLeftY, controller::getLeftX, controller::getRightX));
    }

    else{
      swerve.setDefaultCommand(swerve.driveFieldRelativeRedCommand(controller::getLeftY, controller::getLeftX, controller::getRightX));
    }


  }

  public void teleopInit() {
    colour = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
