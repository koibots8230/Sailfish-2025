package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
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
        elevator.setPositionCommand(ElevatorConstants.LOWER_ALGAE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.LOWER_ALGAE_POSITION)),
        endEffector.setVelocityCommand(EndEffectorConstants.ALGAE_REMOVAL_SPEED));
  }

  public static Command removeL3Algae(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.UPPER_ALGAE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.UPPER_ALGAE_POSITION)),
        endEffector.setVelocityCommand(EndEffectorConstants.ALGAE_REMOVAL_SPEED));
  }
}
