package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimbCommands {
  public static Command prepClimb(Climber climber) {
    return climber.setAngleCommand(ClimberConstants.PREP_POSITION);
  }

  public static Command fastClimb(Climber climber) {
    return new SequentialCommandGroup(
      climber.setSpeedCommand(ClimberConstants.HIGH_SPEED),
      climber.setAngleCommand(ClimberConstants.CLIMB_POSITION));
  }

  public static Command slowClimb(Climber climber) {
    return new SequentialCommandGroup(
      climber.setSpeedCommand(ClimberConstants.LOW_SPEED),
      climber.setAngleCommand(ClimberConstants.CLIMB_POSITION));
  }


  public static Command resetClimber(Climber climber) {
    return climber.setAngleCommand(ClimberConstants.START_POSITION);
  }
}
