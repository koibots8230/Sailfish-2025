// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {
  private final AutoFactory autoFactory;
  private final AutoChooser autochooser;

  private final Swerve swerve;
  private final Elevator elevator;
  private final EndEffector endEffector;

  // SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final Intake intake;
  private final IntakePivot intakePivot;

  private final Indexer indexer;

  @NotLogged private final CommandXboxController xboxController;
  private boolean isBlue;

  public RobotContainer() {
    swerve = new Swerve();
    elevator = new Elevator();
    endEffector = new EndEffector();
    intake = new Intake();
    intakePivot = new IntakePivot();
    indexer = new Indexer();

    autochooser = new AutoChooser();

    NamedCommands.registerCommand(
        "LevelTwoScoreCommand", ScoreCommands.levelTwo(elevator, endEffector));
    NamedCommands.registerCommand(
        "LevelThreeScoreCommand", ScoreCommands.levelThree(elevator, endEffector));
    NamedCommands.registerCommand(
        "IntakeCommand",
        IntakeCommands.intakeCommand(intake, intakePivot, indexer, elevator, endEffector));

    xboxController = new CommandXboxController(0);

    autoFactory =
        new AutoFactory(
            swerve::getEstimatedPosition,
            swerve::setOdometry,
            swerve::followTerjectory,
            isBlue,
            swerve);

    Command myTjectory = autoFactory.trajectoryCmd("myTrajectory");

    autoFactory.bind("Score Level Three", scoreLevelThree());
    autoFactory.bind("Score Level Two", scoreLevelTwo());

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);

    autochooser.addCmd("Score Level Three", this::scoreLevelThree);
    autochooser.addCmd("Score Level Two", this::scoreLevelTwo);

    SmartDashboard.putData(autochooser);
    RobotModeTriggers.autonomous().whileTrue(autochooser.selectedCommandScheduler());

    configureBindings();
    defualtCommands();
  }

  public Command scoreLevelThree() {
    return ScoreCommands.levelThree(elevator, endEffector);
  }

  public Command scoreLevelTwo() {
    return ScoreCommands.levelTwo(elevator, endEffector);
  }

  private void configureBindings() {

    Trigger zero = xboxController.b();
    zero.onTrue(swerve.zeroGyroCommand(isBlue));

    Trigger spinIntake = new Trigger(xboxController.rightTrigger());
    spinIntake.onTrue(
        IntakeCommands.intakeCommand(intake, intakePivot, indexer, elevator, endEffector));
    spinIntake.onFalse(IntakeCommands.intakeStop(intake, indexer, intakePivot));

    Trigger reverseIntake = new Trigger(xboxController.leftTrigger());
    reverseIntake.onTrue(
        IntakeCommands.reverseCommand(intake, intakePivot, indexer, elevator, endEffector));
    reverseIntake.onFalse(IntakeCommands.intakeStop(intake, indexer, intakePivot));

    Trigger gotoLevelThree = new Trigger(xboxController.y());
    gotoLevelThree.onTrue(ScoreCommands.levelThree(elevator, endEffector));
    gotoLevelThree.onFalse(ScoreCommands.basePosition(elevator, endEffector));

    Trigger gotoLevelTwo = new Trigger(xboxController.a());
    gotoLevelTwo.onTrue(ScoreCommands.levelTwo(elevator, endEffector));
    gotoLevelTwo.onFalse(ScoreCommands.basePosition(elevator, endEffector));
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
  //   return null;
  // }
}
