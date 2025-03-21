package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.State;

public class ClimbCommands {
  public static Command prepClimb(Climber climber, LED LED) {
    return Commands.sequence(
        climber.setAngleCommand(ClimberConstants.PREP_POSITION),
        LED.LEDCommand(State.cDeploying),
        Commands.waitUntil(climber::atSetpoint),
        LED.LEDCommand(State.cDeployed));
  }

  public static Command climb(Climber climber, LED LED) {
    return Commands.sequence(
        climber.setAngleCommand(ClimberConstants.CLIMB_POSITION),
        LED.LEDCommand(State.climbing),
        Commands.waitUntil(climber::atSetpoint),
        LED.LEDCommand(State.tada));
  }

  public static Command resetClimber(Climber climber) {
    return climber.setAngleCommand(ClimberConstants.START_POSITION);
  }
}
