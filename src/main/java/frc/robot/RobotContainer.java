// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.ReefAlignState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {

  private final Swerve swerve;
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;
  @NotLogged private final SendableChooser<String> sendableChooser = new SendableChooser<>();

  // private final AutoRoutines autoRoutines;
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
    autoFactory =
        new AutoFactory(
            swerve::getEstimatedPosition,
            swerve::setOdometry,
            swerve::followTrajectory,
            true,
            swerve);
    elevator = new Elevator();
    endEffector = new EndEffector();
    intake = new Intake();
    intakePivot = new IntakePivot();
    indexer = new Indexer();

    autoChooser = new AutoChooser();

    vision =
        new Vision(
            swerve::getEstimatedPosition, swerve::getGyroAngle, swerve::addVisionMeasurement);

    xboxController = new CommandXboxController(0);

    //  chooser = AutoBuilder.buildAutoChooser();

    autoChooser.addRoutine("Score Front Left Right Reef", this::scoreFrontLeftRightReefRoutine);
    autoChooser.addRoutine("move stright tune test", this::moveStrightAutoTuneTest);
    autoChooser.addRoutine("rotate tune test", this::rotateAutoTuneTest);
    SmartDashboard.putData("auto choices", autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    configureBindings();
    defualtCommands();
  }

  private AutoRoutine scoreFrontLeftRightReefRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory driveToFrontLeftRightReef = routine.trajectory("ScoreFrontLeftRightReef");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                driveToFrontLeftRightReef.resetOdometry(), driveToFrontLeftRightReef.cmd()));

    driveToFrontLeftRightReef.done().onTrue(ScoreCommands.levelTwo(elevator, endEffector));

    return routine;
  }

  private AutoRoutine moveStrightAutoTuneTest() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory MvST = routine.trajectory("MvST");

    routine.active().onTrue(Commands.sequence(MvST.resetOdometry(), MvST.cmd()));

    return routine;
  }

  private AutoRoutine rotateAutoTuneTest() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory rotate = routine.trajectory("rotate");

    routine.active().onTrue(Commands.sequence(rotate.resetOdometry(), rotate.cmd()));

    return routine;
  }

  private void configureBindings() {

    // Trigger zero = xboxController.b();
    // zero.onTrue(swerve.zeroGyroCommand(isBlue));

    Trigger spinIntake = new Trigger(xboxController.rightTrigger());
    spinIntake.onTrue(
        IntakeCommands.intakeCommand(intake, intakePivot, indexer, elevator, endEffector));
    spinIntake.onFalse(IntakeCommands.intakeStop(intake, indexer, intakePivot, endEffector));

    Trigger reverseIntake = new Trigger(xboxController.leftTrigger());
    reverseIntake.onTrue(
        IntakeCommands.reverseCommand(intake, intakePivot, indexer, elevator, endEffector));
    reverseIntake.onFalse(IntakeCommands.intakeStop(intake, indexer, intakePivot, endEffector));

    Trigger gotoLevelThree = new Trigger(xboxController.y());
    gotoLevelThree.onTrue(
        Commands.sequence(
            ScoreCommands.levelThree(elevator, endEffector),
            swerve.setReefAlignStateCommand(ReefAlignState.disabled)));
    gotoLevelThree.onFalse(ScoreCommands.basePosition(elevator, endEffector));

    Trigger gotoLevelTwo = new Trigger(xboxController.a());
    gotoLevelTwo.onTrue(
        Commands.sequence(
            ScoreCommands.levelTwo(elevator, endEffector),
            swerve.setReefAlignStateCommand(ReefAlignState.disabled)));
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

  public Command getAutonomousCommand() {
    if (sendableChooser.getSelected() == "TEST_ROUTINE") {
      return Commands.sequence(
          autoFactory.resetOdometry("Test2"), autoFactory.trajectoryCmd("Test2"));
    }

    if (sendableChooser.getSelected() == "FRONT_LEFT_RIGHT_REEF") {
      return Commands.sequence(
          autoFactory.resetOdometry("ScoreFrontLeftRightReef"),
          autoFactory.trajectoryCmd("ScoreFrontLeftRightReef"),
          autoFactory.trajectoryCmd("SFLRRf"));
    }
    return null;
    // return autoChooser.selectedCommand();
    // return Commands.sequence(autoFactory.resetOdometry("Test2"),
    //   autoFactory.trajectoryCmd("Test2"));
  }

  public void autonomousInit() {
    isBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.setIsBlue(isBlue);
  }
}
