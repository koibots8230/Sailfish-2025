package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ScoreCommands {

  public static Command levelThree(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.L3_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.L3_POSITION)),
        endEffector.outtakeCommand(),
        elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION));
  }

  public static Command levelTwo(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.L2_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.L2_POSITION)),
        endEffector.outtakeCommand(),
        elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION));
  }

  public static Command basePosition(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION),
        endEffector.setVelocityCommand(0));
  }

  public static Command removeL2Algae(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(Meters.of(0.65)),
        Commands.waitUntil(() -> elevator.atPosition(Meters.of(0.65))),
        endEffector.setVelocityCommand(1500));
  }

  public static Command removeL3Algae(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(Meters.of(1.5)),
        Commands.waitUntil(() -> elevator.atPosition(Meters.of(1.5))),
        endEffector.setVelocityCommand(1500));
  }
}
