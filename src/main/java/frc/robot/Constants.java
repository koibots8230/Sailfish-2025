package frc.robot;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;



public class Constants {

  public static class RobotConstants {

    public static final Distance WIDTH = Inches.of(26.0);
    public static final Distance LENGTH = Inches.of(26.0);

    public static  final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(RobotConstants.LENGTH.divide(2), RobotConstants.WIDTH.divide(2)),
      new Translation2d(RobotConstants.LENGTH.divide(2), RobotConstants.WIDTH.divide(-2)),
      new Translation2d(RobotConstants.LENGTH.divide(-2), RobotConstants.WIDTH.divide(2)),
      new Translation2d(RobotConstants.LENGTH.divide(-2), RobotConstants.WIDTH.divide(-2))
      );    
    
  }

  // STEP 6: Add a LinearVelocity MAX_SPEED and a AngularVelocity MAX_ANGULAR_VELOCITY
  public static class SwerveConstants {

    public static final double DEADBAND = 0.05;

    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.25);

    public static final AngularVelocity MAX_ROTATION = RadiansPerSecond.of(2 * Math.PI);

    public static final double LEFT_STICK_SCAILING = 2;

    public static final double RIGHT_STICK_SCAILING = 3;
  }
}
