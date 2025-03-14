// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final Elevator elevator;
  private final EndEffector endEffector;

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
            swerve::getEstimatedPosition,
            swerve::getGyroAngle,
            swerve::addVisionMeasurement,
            swerve::getIsBlue);

    xboxController = new CommandXboxController(0);

    //  chooser = AutoBuilder.buildAutoChooser();

    autoChooser.addRoutine("Score Front Left Right Reef", this::SeFLRRf);
    autoChooser.addRoutine("move stright tune test", this::MeSATTt);
    autoChooser.addRoutine("rotate tune test", this::ReATTt);
    autoChooser.addRoutine("Score Front Right Left Reef", this::SeFRLRf);
    autoChooser.addRoutine("Circle the reef for no rasin", this::torpedo);
    SmartDashboard.putData("auto choices", autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    configureBindings();
    defualtCommands();
  }

  private AutoRoutine torpedo() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory torpedoTy = routine.trajectory("torpedo");

    routine.active().onTrue(Commands.sequence(torpedoTy.resetOdometry(), torpedoTy.cmd()));

    torpedoTy.done().onTrue(ScoreCommands.levelTwo(elevator, endEffector));

    return routine;
  }

  private AutoRoutine SeFLRRf() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory driveToFrontLeftRightReef = routine.trajectory("SeFLRRf");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                driveToFrontLeftRightReef.resetOdometry(),
                Commands.parallel(
                    driveToFrontLeftRightReef.cmd(),
                    ScoreCommands.algieRemoverCommand(elevator, endEffector))));

    driveToFrontLeftRightReef.done().onTrue(ScoreCommands.levelTwo(elevator, endEffector));

    return routine;
  }

  private AutoRoutine SeFRLRf() {
    AutoRoutine routine = autoFactory.newRoutine("txai");

    AutoTrajectory SeFLRRfTy = routine.trajectory("SeFLRRf");

    routine.active().onTrue(Commands.sequence(SeFLRRfTy.resetOdometry(), SeFLRRfTy.cmd()));

    SeFLRRfTy.done().onTrue(ScoreCommands.levelTwo(elevator, endEffector));

    return routine;
  }

  private AutoRoutine MeSATTt() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory MeST = routine.trajectory("MeST");

    routine.active().onTrue(Commands.sequence(MeST.resetOdometry(), MeST.cmd()));

    return routine;
  }

  private AutoRoutine ReATTt() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory rotate = routine.trajectory("Re");

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

  public void autonomousInit() {
    isBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.setIsBlue(isBlue);
  }
}
