package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ScoreCommands {

  public static Command levelThree(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.L3_SETPOINT),
        Commands.waitUntil(() -> elevator.positionIsInRAnge(ElevatorConstants.L3_SETPOINT)),
        endEffector.outtakeCommand(),
        elevator.setPositionCommand(ElevatorConstants.INTAKE_SETPOINT));
  }

  public static Command levelTwo(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.L2_SETPOINT),
        Commands.waitUntil(() -> elevator.positionIsInRAnge(ElevatorConstants.L2_SETPOINT)),
        endEffector.outtakeCommand(),
        elevator.setPositionCommand(ElevatorConstants.INTAKE_SETPOINT));
  }

  public static Command basePosition(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
        elevator.setPositionCommand(ElevatorConstants.INTAKE_SETPOINT),
        endEffector.setVelocityCommand(0));
  }
}
