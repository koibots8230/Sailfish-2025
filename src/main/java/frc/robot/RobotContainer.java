// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.*;

@Logged
public class RobotContainer {

  private final Swerve swerve;
  private final Elevator elevator;
  private final Intake intake;
  private final IntakePivot intakePivot;

  @NotLogged private final CommandXboxController xboxController;


  public RobotContainer() {
    swerve = new Swerve();
    elevator = new Elevator();
    intake = new Intake();
    intakePivot = new IntakePivot();

    xboxController = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            xboxController::getLeftY, xboxController::getLeftX, xboxController::getRightX));

    //Trigger L1 = xboxController.a();
    //L1.onTrue(elevator.setPositionCommand(ElevatorConstants.L1_SETPOINT));
    Trigger L2 = xboxController.b();
    L2.onTrue(elevator.setPositionCommand(ElevatorConstants.L2_SETPOINT));
    Trigger L3 = xboxController.x();
    L3.onTrue(elevator.setPositionCommand(ElevatorConstants.L3_SETPOINT));
    Trigger L4 = xboxController.y();
    L4.onTrue(elevator.setPositionCommand(ElevatorConstants.L4_SETPOINT));
    Trigger ElevatorDown = xboxController.rightBumper();
    ElevatorDown.onTrue(elevator.setPositionCommand(ElevatorConstants.START_SETPOINT));

    Trigger spinIntake = new Trigger(xboxController.b());
    spinIntake.onTrue(intake.IntakeCommand(IntakeConstants.INTAKE_VELOCITY));
    spinIntake.onFalse(intake.IntakeCommand(AngularVelocity.ofBaseUnits(0, RPM)));

    Trigger intakePivotOut = new Trigger(xboxController.a());
    intakePivotOut.onTrue(intakePivot.moveIntakePivotCommand(IntakePivotConstants.OUT_POSITION));

  }




  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
