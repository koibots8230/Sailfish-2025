package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class ScoreCommand {
    
    public static Command levelThreeScore(Elevator elevator, EndEffector endEffector){
    return Commands.sequence(
    elevator.setPositionCommand(ElevatorConstants.L3_SETPOINT),
    Commands.waitUntil(() -> elevator.getPosition(ElevatorConstants.L3_SETPOINT)),
    endEffector.outtakeCommand()
    );
    }

    public static Command levelTwoScore(Elevator elevator, EndEffector endEffector) {
    return Commands.sequence(
    elevator.setPositionCommand(ElevatorConstants.L2_SETPOINT),
    Commands.waitUntil(() -> elevator.getPosition(ElevatorConstants.L2_SETPOINT)),
    endEffector.outtakeCommand()
    );
    }

    public static Command basePosition(Elevator elevator) {
    return Commands.sequence(
    elevator.setPositionCommand(ElevatorConstants.START_SETPOINT)
    );
    }

}
