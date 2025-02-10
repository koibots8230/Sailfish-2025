// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
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

  // SendableChooser<Command> chooser = new SendableChooser<>();

  @NotLogged private final CommandXboxController xboxController;
  private boolean isBlue;

  public RobotContainer() {
    swerve = new Swerve();
    elevator = new Elevator();
    endEffector = new EndEffector();

    xboxController = new CommandXboxController(0);

    //  chooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    defualtCommands();
  }

  private void configureBindings() {

    Trigger testEffector = xboxController.x();
    testEffector.onTrue(endEffector.setVelocityCommand(EndEffectorConstants.INTAKE_SPEED));
    testEffector.onFalse(endEffector.setVelocityCommand(0));

    Trigger intakeEffector = xboxController.a();
    intakeEffector.onTrue(endEffector.intakeCommand());
    intakeEffector.onFalse(endEffector.setVelocityCommand(0));

    Trigger outtakeEffector = xboxController.b();
    outtakeEffector.onTrue(endEffector.outtakeCommand());
    outtakeEffector.onFalse(endEffector.setVelocityCommand(0));

    Trigger L1 = xboxController.a(); // TODO CHANGE VALUE TO SOMTHING ELSE SHOULD NOT HAVE TWO A
    L1.onTrue(elevator.setPositionCommand(ElevatorConstants.L1_SETPOINT));

    Trigger ElevatorDown = xboxController.y();
    ElevatorDown.onTrue(elevator.setPositionCommand(ElevatorConstants.START_SETPOINT));

    Trigger zero = xboxController.y();

    zero.onTrue(swerve.zeroGyroCommand(isBlue));
  }

  private void defualtCommands() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            xboxController::getLeftY, xboxController::getLeftX, xboxController::getRightX));
  }

  public void teleopInit() {
    isBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.setIsBlue(isBlue);
  }

  // public Command getAutonomousCommand() {
  //   return chooser.getSelected();
  // }
}
