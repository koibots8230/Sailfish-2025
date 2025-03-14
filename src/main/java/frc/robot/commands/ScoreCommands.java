package frc.robot.commands;

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

  public static Command algieRemoverCommand(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        endEffector.setVelocityCommand(-100),
        Commands.waitSeconds(.1),
        endEffector.setVelocityCommand(0));
  }
}
