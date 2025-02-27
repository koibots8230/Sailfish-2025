// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.lib.util.ReefAlignState;
import frc.robot.commands.*;

@Logged
public class RobotContainer {

  private final Swerve swerve;
  private final Elevator elevator;
  private final EndEffector endEffector;

  // SendableChooser<Command> chooser = new SendableChooser<>();
  private final Intake intake;
  private final IntakePivot intakePivot;

  private final Indexer indexer;

  private final Vision vision;

  private final CommandXboxController xboxController;
  private boolean isBlue;

  public RobotContainer() {
    swerve = new Swerve();
    elevator = new Elevator();
    endEffector = new EndEffector();
    intake = new Intake();
    intakePivot = new IntakePivot();
    indexer = new Indexer();

    vision =
        new Vision(
            swerve::getEstimatedPosition, swerve::getGyroAngle, swerve::addVisionMeasurement);

    xboxController = new CommandXboxController(0);

    //  chooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    defualtCommands();
  }

  private void configureBindings() {

    // Trigger zero = xboxController.b();
    // zero.onTrue(swerve.zeroGyroCommand(isBlue));

    Trigger spinIntake = new Trigger(xboxController.rightTrigger());
    spinIntake.onTrue(
        IntakeCommands.intakeCommand(intake, intakePivot, indexer, elevator, endEffector));
    spinIntake.onFalse(IntakeCommands.intakeStop(intake, indexer, intakePivot));

    Trigger reverseIntake = new Trigger(xboxController.leftTrigger());
    reverseIntake.onTrue(
        IntakeCommands.reverseCommand(intake, intakePivot, indexer, elevator, endEffector));
    reverseIntake.onFalse(IntakeCommands.intakeStop(intake, indexer, intakePivot));

    Trigger gotoLevelThree = new Trigger(xboxController.y());
    gotoLevelThree.onTrue(
      Commands.sequence(ScoreCommands.levelThree(elevator, endEffector), swerve.setReefAlignStateCommand(ReefAlignState.disabled)));
    gotoLevelThree.onFalse(ScoreCommands.basePosition(elevator, endEffector));

    Trigger gotoLevelTwo = new Trigger(xboxController.a());
    gotoLevelTwo.onTrue(
      Commands.sequence(ScoreCommands.levelTwo(elevator, endEffector), swerve.setReefAlignStateCommand(ReefAlignState.disabled)));
    gotoLevelTwo.onFalse(ScoreCommands.basePosition(elevator, endEffector));

    Trigger alignRight = xboxController.rightBumper();
    alignRight.onTrue(swerve.setReefAlignStateCommand(ReefAlignState.rightSide));

    Trigger alignLeft = xboxController.leftBumper();
    alignLeft.onTrue(swerve.setReefAlignStateCommand(ReefAlignState.leftSide));
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
