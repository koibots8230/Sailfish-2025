package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimbCommands {
  public static Command prepClimb(Climber climber) {
    return climber.setAngleCommand(ClimberConstants.PREP_POSITION);
  }

  public static Command climb(Climber climber) {
    return climber.setAngleCommand(ClimberConstants.CLIMB_POSITION);
  }

  public static Command resetClimber(Climber climber) {
    return climber.setAngleCommand(ClimberConstants.START_POSITION);
  }
}
