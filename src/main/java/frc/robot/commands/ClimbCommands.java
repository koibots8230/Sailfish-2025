package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED.State;

public class ClimbCommands {
  public static Command prepClimb(Climber climber) {
    return Commands.parallel(
      climber.setAngleCommand(ClimberConstants.PREP_POSITION),
      LEDCommand(State.cDeploying));
  }

  public static Command climb(Climber climber) {
    return Commands.parallel(
      climber.setAngleCommand(ClimberConstants.CLIMB_POSITION),
      LEDCommand(State.climbing));
  }

  public static Command resetClimber(Climber climber) {
    return climber.setAngleCommand(ClimberConstants.START_POSITION);
  }
}
