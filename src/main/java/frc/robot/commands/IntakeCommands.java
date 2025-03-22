package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.State;

public class IntakeCommands {

  public static Command intakeCommand(
      Intake intake,
      IntakePivot intakePivot,
      Indexer indexer,
      Elevator elevator,
      EndEffector endEffector,
      LED LED) {
    return Commands.sequence(
        LED.LEDCommand(State.intake),
        elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.INTAKE_POSITION)),
        Commands.parallel(
            intake.setVelocityCommand(IntakeConstants.INTAKE_VELOCITY),
            intakePivot.setPositionCommand(IntakePivotConstants.OUT_POSITION),
            indexer.setVelocityCommand(
                IndexerConstants.TOP_INDEX_VELOCITY, IndexerConstants.BOTTOM_INDEX_VELOCITY),
            endEffector.intakeCommand()),
        Commands.parallel(intakeStop(intake, indexer, intakePivot, endEffector, LED)));
  }

  public static Command intakeStop(
      Intake intake, Indexer indexer, IntakePivot intakePivot, EndEffector endEffector, LED LED) {
    return Commands.parallel(
        LED.phaseCommand(),
        intake.setVelocityCommand(0),
        indexer.setVelocityCommand(0, 0),
        intakePivot.setPositionCommand(IntakePivotConstants.IN_POSITION),
        endEffector.setVelocityCommand(0));
  }

  public static Command reverseCommand(
      Intake intake,
      IntakePivot intakePivot,
      Indexer indexer,
      Elevator elevator,
      EndEffector endEffector,
      LED LED) {
    return Commands.sequence(
        LED.LEDCommand(State.intake),
        elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.INTAKE_POSITION)),
        intakePivot.setPositionCommand(IntakePivotConstants.OUT_POSITION),
        endEffector.setVelocityCommand(EndEffectorConstants.OUTTAKE_SPEED),
        Commands.waitUntil(() -> intakePivot.atSetpoint(IntakePivotConstants.OUT_POSITION)),
        Commands.parallel(
            intake.setVelocityCommand(IntakeConstants.REVERSE_INTAKE_VELOCITY),
            indexer.setVelocityCommand(
                IndexerConstants.TOP_REVERSE_VELOCITY, IndexerConstants.BOTTOM_REVERSE_VELOCITY)),
        Commands.parallel(intakeStop(intake, indexer, intakePivot, endEffector, LED)));
  }

  public static Command reverseEffectorIndexerCommand(
      Indexer indexer, Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.INTAKE_POSITION)),
        endEffector.setVelocityCommand(-EndEffectorConstants.INTAKE_SPEED),
        indexer.setVelocityCommand(
            IndexerConstants.TOP_REVERSE_VELOCITY, IndexerConstants.BOTTOM_REVERSE_VELOCITY));
  }
}
