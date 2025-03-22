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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.EndEffectorState;
import frc.lib.util.ReefAlignState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
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
    indexer = new Indexer();
    climber = new Climber();
    LED = new LED(endEffector::hasCoral);

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
        IntakeCommands.intakeCommand(intake, intakePivot, indexer, elevator, endEffector, LED));
    spinIntake.onFalse(IntakeCommands.intakeStop(intake, indexer, intakePivot, endEffector, LED));

    Trigger reverseIntake = new Trigger(xboxController.leftTrigger());
    reverseIntake.onTrue(
        IntakeCommands.reverseCommand(intake, intakePivot, indexer, elevator, endEffector, LED));
    reverseIntake.onFalse(
        Commands.sequence(
            IntakeCommands.intakeStop(intake, indexer, intakePivot, endEffector, LED),
            Commands.waitSeconds(0.15),
            Commands.either(
                Commands.none(),
                endEffector.setStateCommand(EndEffectorState.noCoral),
                endEffector::hasCoral)));

    Trigger reverseEffectorIndexer = xboxController.x();
    reverseEffectorIndexer.onTrue(
        IntakeCommands.reverseEffectorIndexerCommand(indexer, elevator, endEffector));
    reverseEffectorIndexer.onFalse(
        IntakeCommands.intakeStop(intake, indexer, intakePivot, endEffector, LED));

    Trigger gotoLevelThree = new Trigger(xboxController.y());
    gotoLevelThree.onTrue(
        Commands.parallel(
            ScoreCommands.levelThree(elevator, endEffector),
            swerve.setReefAlignStateCommand(ReefAlignState.disabled)));
    gotoLevelThree.onFalse(ScoreCommands.basePosition(elevator, endEffector));

    Trigger gotoLevelTwo = new Trigger(xboxController.a());
    gotoLevelTwo.onTrue(
        Commands.parallel(
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

    Trigger cancelAlign = xboxController.b();
    cancelAlign.onTrue(swerve.setReefAlignStateCommand(ReefAlignState.disabled));

    Trigger prepClimb = new Trigger(() -> operatorPad.getRawButton(5));
    prepClimb.onTrue(ClimbCommands.prepClimb(climber, LED));

    Trigger climb = new Trigger(() -> operatorPad.getRawButton(6));
    climb.onTrue(ClimbCommands.climb(climber, LED));

    Trigger resetClimb = new Trigger(() -> operatorPad.getRawButton(8));
    resetClimb.onTrue(ClimbCommands.resetClimber(climber));

    Trigger xMode = new Trigger(() -> operatorPad.getRawButton(3));
    xMode.onTrue(LED.XModeCommand());
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

  public void autoInit() {
    LED.setAutoCommand();
  }

  public void teleopInit() {
    this.defualtCommands();
    LED.setTeleopCommand().schedule();

    endEffector.setDefaultCommand(endEffector.holdCoralCommand());
  }

  private AutoRoutine centerScore() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory leave = routine.trajectory("MiddleScore");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                leave.resetOdometry(),
                Commands.parallel(leave.cmd(), endEffector.releaseAlgaeRemover()),
                ScoreCommands.levelTwo(elevator, endEffector)));

    return routine;
  }

  private AutoRoutine SeFLRRf() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory driveToFrontLeftRightReef = routine.trajectory("SeFLRRf");
    AutoTrajectory moveToMiddleReef = routine.trajectory("FLRRfMeBLMRf");
    AutoTrajectory moveToBackReef = routine.trajectory("BLRfMeBMMRf");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                driveToFrontLeftRightReef.resetOdometry(),
                endEffector.releaseAlgaeRemover(),
                Commands.print("HIIII"),
                driveToFrontLeftRightReef.cmd()));

    driveToFrontLeftRightReef
        .done()
        .onTrue(
            Commands.sequence(
                ScoreCommands.levelTwo(elevator, endEffector),
                Commands.deadline(
                    new WaitCommand(2), ScoreCommands.removeL3Algae(elevator, endEffector)),
                ScoreCommands.basePosition(elevator, endEffector),
                moveToMiddleReef.cmd()));

    moveToMiddleReef
        .done()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(2), ScoreCommands.removeL2Algae(elevator, endEffector)),
                ScoreCommands.basePosition(elevator, endEffector),
                moveToBackReef.cmd()));

    moveToBackReef
        .done()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(2), ScoreCommands.removeL3Algae(elevator, endEffector)),
                ScoreCommands.basePosition(elevator, endEffector)));

    return routine;
  }

  private AutoRoutine SeFRLRf() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory SeFLRRfTy = routine.trajectory("SeFLRRf");
    AutoTrajectory frontToBackRight = routine.trajectory("FRLRfMeBRMRf");
    AutoTrajectory backRightToBackMiddle = routine.trajectory("BRMRfMeBMMRf");

    routine.active().onTrue(Commands.sequence(SeFLRRfTy.resetOdometry(), SeFLRRfTy.cmd()));

    SeFLRRfTy.done()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(1), ScoreCommands.removeL3Algae(elevator, endEffector)),
                ScoreCommands.levelTwo(elevator, endEffector),
                frontToBackRight.cmd()));
    frontToBackRight
        .done()
        .onFalse(
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(1), ScoreCommands.removeL3Algae(elevator, endEffector)),
                backRightToBackMiddle.cmd()));
    backRightToBackMiddle
        .done()
        .onTrue(
            Commands.deadline(
                new WaitCommand(1), ScoreCommands.removeL3Algae(elevator, endEffector)));

    return routine;
  }

  private AutoRoutine MeSATTt() {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    AutoTrajectory MeST = routine.trajectory("MeST");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                elevator.setPositionCommand(ElevatorConstants.L2_POSITION),
                MeST.resetOdometry(),
                MeST.cmd()));

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
                    endEffector.setVelocityCommand(-250),
                    Commands.waitSeconds(0.3),
                    endEffector.setVelocityCommand(0)),
                leave.resetOdometry(),
                leave.cmd(),
                Commands.sequence(
                    elevator.setPositionCommand(ElevatorConstants.L2_ALGAE_POSITION),
                    Commands.waitUntil(
                        () -> elevator.atPosition(ElevatorConstants.L2_ALGAE_POSITION)),
                    endEffector.setVelocityCommand(EndEffectorConstants.ALGAE_REMOVAL_SPEED)),
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
                leave.resetOdometry(),
                Commands.parallel(endEffector.releaseAlgaeRemover(), leave.cmd())));

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
