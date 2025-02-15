// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {

  private final Swerve swerve;
  private final Elevator elevator;

  private final EndEffector endEffector;

  // SendableChooser<Command> chooser = new SendableChooser<>();
  private final Intake intake;
  private final IntakePivot intakePivot;

  @NotLogged private final CommandXboxController xboxController;
  private boolean isBlue;

  public RobotContainer() {
    swerve = new Swerve();
    elevator = new Elevator();
    endEffector = new EndEffector();
    intake = new Intake();
    intakePivot = new IntakePivot();

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

    Trigger spinIntake = new Trigger(xboxController.rightTrigger());
    Trigger reverseIntake = new Trigger(xboxController.leftTrigger());

    // spinIntake.onTrue(intake.IntakeCommand(IntakeConstants.INTAKE_VELOCITY));
    // spinIntake.onFalse(intake.IntakeCommand(AngularVelocity.ofBaseUnits(0, RPM)));
    spinIntake.onTrue(
        Commands.parallel(
            intakePivot.moveIntakePivotCommand(IntakePivotConstants.OUT_POSITION),
            intake.IntakeCommand(IntakeConstants.INTAKE_VELOCITY)));
    spinIntake.onFalse(
        Commands.sequence(
            intake.IntakeCommand(AngularVelocity.ofBaseUnits(0, RPM)),
            intakePivot.moveIntakePivotCommand(IntakePivotConstants.IN_POSITION)));

    // reverseIntake.onTrue(intake.IntakeCommand(IntakeConstants.REVERSE_INTAKE_VELOCITY));
    // reverseIntake.onFalse(intake.IntakeCommand(AngularVelocity.ofBaseUnits(0, RPM)));
    reverseIntake.onTrue(
        Commands.sequence(
            intakePivot.moveIntakePivotCommand(IntakePivotConstants.OUT_POSITION),
            intake.IntakeCommand(IntakeConstants.REVERSE_INTAKE_VELOCITY)));
    reverseIntake.onFalse(
        Commands.parallel(
            intake.IntakeCommand(AngularVelocity.ofBaseUnits(0, RPM)),
            intakePivot.moveIntakePivotCommand(IntakePivotConstants.IN_POSITION)));

    // Trigger intakePivotOut = new Trigger(xboxController.b());
    // intakePivotOut.onTrue(intakePivot.moveIntakePivotCommand(IntakePivotConstants.OUT_POSITION));
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
