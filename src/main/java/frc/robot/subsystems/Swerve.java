package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {

  private Pose2d estimatedPose;

  public Swerve() {

    // NOTE: An empty constructor will set the robot at
    // (0,0) on the field,
    // facing to the right.

    estimatedPose = new Pose2d();
  }

  /**
   * Drives the robot for the current duty cycle.
   *
   * @param x field-relative X linear velocity in meters/s
   * @param y field-relative Y linear velocity in meters/s
   * @param omega field-relative angular velocity in radians/s
   */
  private void driveFieldRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    estimatedPose =
        estimatedPose.plus(
            new Transform2d(
                x.times(SwerveConstants.SWERVE_UPDATE_PERIOD),
                y.times(SwerveConstants.SWERVE_UPDATE_PERIOD),
                new Rotation2d(omega.times(SwerveConstants.SWERVE_UPDATE_PERIOD))));
  }

  /**
   * Returns a Command that passes the input parameters into {@link
   * #driveFieldRelative(LinearVelocity, LinearVelocity, AngularVelocity)}.
   *
   * @param x field-relative X with range of -1.0 to 1.0
   * @param y field-relative Y with range of -1.0 to 1.0
   * @param omega field-relative omega with range of -1.0 to 1.0
   */
  public Command driveFieldRelativeCommand(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () ->
            driveFieldRelative(
                SwerveConstants.MAX_SPEED.times(-x.getAsDouble()),
                SwerveConstants.MAX_SPEED.times(-y.getAsDouble()),
                SwerveConstants.MAX_ANGULAR_VELOCITY.times(-omega.getAsDouble())),
        this);
  }
}

