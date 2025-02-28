package frc.robot.routines;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {

  public AutoRoutine testRoutine(AutoFactory autoFactory) {
    AutoRoutine routine = autoFactory.newRoutine("taxi");

    // Load the routine's trajectories
    AutoTrajectory driveToMiddle = routine.trajectory("Test");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    return routine;
  }
}
