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
import edu.wpi.first.wpilibj.GenericHID;
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

  private final Climber climber;

  private final Vision vision;

  private final LED LED;

  private final CommandXboxController xboxController;
  private final GenericHID operatorPad;

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
    LED = new LED();
    indexer = new Indexer();
    climber = new Climber();

    autoChooser = new AutoChooser();

    vision =
        new Vision(
            swerve::getEstimatedPosition,
            swerve::getGyroAngle,
            swerve::addVisionMeasurement,
            swerve::getIsBlue);

    xboxController = new CommandXboxController(0);
    operatorPad = new GenericHID(1);

    autoChooser.addRoutine("Score Front Left Right Reef", this::SeFLRRf);
    autoChooser.addRoutine("move stright tune test", this::MeSATTt);
    autoChooser.addRoutine("rotate tune test", this::ReATTt);
    autoChooser.addRoutine("Leave Left", this::leaveLeft);
    autoChooser.addRoutine("Leave Center", this::leaveCenter);
    autoChooser.addRoutine("Leave Right", this::leaveRight);
    SmartDashboard.putData("auto choices", autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    configureBindings();
    defualtCommands();
    setupTestMode();
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

    Trigger removeL2Algae = xboxController.povDown();
    removeL2Algae.onTrue(ScoreCommands.removeL2Algae(elevator, endEffector));
    removeL2Algae.onFalse(ScoreCommands.basePosition(elevator, endEffector));

    Trigger removeL3Algae = xboxController.povUp();
    removeL3Algae.onTrue(ScoreCommands.removeL3Algae(elevator, endEffector));
    removeL3Algae.onFalse(ScoreCommands.basePosition(elevator, endEffector));

    Trigger alignRight = xboxController.rightBumper();
    alignRight.onTrue(swerve.setReefAlignStateCommand(ReefAlignState.rightSide));

    Trigger alignLeft = xboxController.leftBumper();
    alignLeft.onTrue(swerve.setReefAlignStateCommand(ReefAlignState.leftSide));

    // Trigger prepClimb = new Trigger(() -> operatorPad.getRawButton(5));
    // prepClimb.onTrue(ClimbCommands.prepClimb(climber));

    // Trigger climb = new Trigger(() -> operatorPad.getRawButton(6));
    // climb.onTrue(ClimbCommands.climb(climber));

    // Trigger resetClimb = new Trigger(() -> operatorPad.getRawButton(7));
    // resetClimb.onTrue(ClimbCommands.resetClimber(climber));
  }

  private void defualtCommands() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            xboxController::getLeftY, xboxController::getLeftX, xboxController::getRightX));
  }

  public void setAlliance() {
    isBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.setIsBlue(isBlue);
  }

  public void teleopInit() {
    this.defualtCommands();
  }

  private AutoRoutine SeFLRRf() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory driveToFrontLeftRightReef = routine.trajectory("SeFLRRf");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                driveToFrontLeftRightReef.resetOdometry(), driveToFrontLeftRightReef.cmd()));

    driveToFrontLeftRightReef.done().onTrue(ScoreCommands.levelTwo(elevator, endEffector));

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

  private AutoRoutine leaveLeft() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory leave = routine.trajectory("LL");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.sequence(
                    endEffector.setVelocityCommand(-300),
                    Commands.waitSeconds(0.1),
                    endEffector.setVelocityCommand(0)),
                leave.resetOdometry(),
                leave.cmd()));

    return routine;
  }

  private AutoRoutine leaveCenter() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory leave = routine.trajectory("LC");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.sequence(
                    endEffector.setVelocityCommand(-300),
                    Commands.waitSeconds(0.1),
                    endEffector.setVelocityCommand(0)),
                leave.resetOdometry(),
                leave.cmd(),
                ScoreCommands.removeL2Algae(elevator, endEffector),
                Commands.waitSeconds(0.75),
                ScoreCommands.basePosition(elevator, endEffector)));

    return routine;
  }

  private AutoRoutine leaveRight() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory leave = routine.trajectory("LR");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.sequence(
                    endEffector.setVelocityCommand(-300),
                    Commands.waitSeconds(0.1),
                    endEffector.setVelocityCommand(0)),
                leave.resetOdometry(),
                leave.cmd()));

    return routine;
  }

  private void setupTestMode() {
    SmartDashboard.putBoolean("SwerveTest/FL", true);
    SmartDashboard.putBoolean("SwerveTest/FR", true);
    SmartDashboard.putBoolean("SwerveTest/BL", true);
    SmartDashboard.putBoolean("SwerveTest/BR", true);

    SmartDashboard.putBoolean("TestSequence/Intake", true);
    SmartDashboard.putBoolean("TestSequence/Intake Pivot", true);

    SmartDashboard.putBoolean("TestSequence/Indexer Top", true);
    SmartDashboard.putBoolean("TestSequence/Indexer Bottom", true);

    SmartDashboard.putBoolean("TestSequence/Elevator", true);

    SmartDashboard.putBoolean("TestSequence/End Effector", true);

    SmartDashboard.putData(
        "TestSequence/SequenceCommand",
        TestCommands.testSequence(swerve, intake, intakePivot, indexer, elevator, endEffector));
  }
}
