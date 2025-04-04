package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.EndEffectorState;
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
        endEffector.setVelocityCommand(0),
        Commands.waitSeconds(0.15),
        Commands.either(
            Commands.none(),
            endEffector.setStateCommand(EndEffectorState.noCoral),
            endEffector::hasCoral));
  }

  public static Command removeL2Algae(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.L2_ALGAE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.L2_ALGAE_POSITION)),
        endEffector.removeAlgaeCommand());
  }

  public static Command levelOne(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.L2_ALGAE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.L2_ALGAE_POSITION)),
        endEffector.outtakeCommand());
  }

  public static Command removeL3Algae(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.L3_ALGAE_POSITION),
        Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.L3_ALGAE_POSITION)),
        endEffector.removeAlgaeCommand());
  }
}
