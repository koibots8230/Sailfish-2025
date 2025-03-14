package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;

public class IntakeCommands {

  public static Command intakeCommand(
      Intake intake,
      IntakePivot intakePivot,
      Indexer indexer,
      Elevator elevator,
      EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.INTAKE_POSITION)),
        Commands.parallel(
            intake.setVeclocityCommand(IntakeConstants.INTAKE_VELOCITY),
            // intakePivot.moveIntakePivotCommand(IntakePivotConstants.OUT_POSITION),
            indexer.setVelocityCommand(IndexerConstants.TOP_INDEX_VELOCITY, IndexerConstants.BOTTOM_INDEX_VELOCITY),
            endEffector.intakeCommand()),
        Commands.parallel(intakeStop(intake, indexer, intakePivot, endEffector)));
  }

  public static Command intakeStop(
      Intake intake, Indexer indexer, IntakePivot intakePivot, EndEffector endEffector) {
    return Commands.parallel(
        intake.setVeclocityCommand(0),
        indexer.setVelocityCommand(0, 0),
        // intakePivot.moveIntakePivotCommand(IntakePivotConstants.IN_POSITION),
        endEffector.setVelocityCommand(0));
  }

  public static Command reverseCommand(
      Intake intake,
      IntakePivot intakePivot,
      Indexer indexer,
      Elevator elevator,
      EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.INTAKE_POSITION)),
        // intakePivot.moveIntakePivotCommand(IntakePivotConstants.OUT_POSITION),
        // Commands.waitUntil(() -> intakePivot.positionIsInRange()),
        Commands.parallel(
            intake.setVeclocityCommand(IntakeConstants.REVERSE_INTAKE_VELOCITY),
            indexer.setVelocityCommand(IndexerConstants.TOP_REVERSE_VELOCITY, IndexerConstants.BOTTOM_REVERSE_VELOCITY),
            endEffector.outtakeCommand()),
        Commands.parallel(intakeStop(intake, indexer, intakePivot, endEffector)));
  }
}
