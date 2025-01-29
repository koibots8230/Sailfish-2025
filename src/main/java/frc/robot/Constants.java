package frc.robot;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;


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

    public static final Time CLOCK_TIME = Second.of(.02);
    
  }

  public static class SwerveConstants {
    public static final PIDGains TURN_PID = new PIDGains.Builder().kp(0.2).build();
    public static final PIDGains DRIVE_PID = new PIDGains.Builder().kp(0.1).build();
    public static final FeedforwardGains TURN_FEEDFORWARD = new FeedforwardGains .Builder().kv(0.2).build();
    public static final FeedforwardGains DRIVE_FEEDFORWARD = new FeedforwardGains .Builder().kv(0.26).build();
    public static final Current TURN_CURRENT_LIMIT = Current.ofBaseUnits(20, Units.Amps);
    public static final Current DRIVE_CURRENT_LIMIT = Current.ofBaseUnits(211, Units.Amps);
    public static final double DRIVE_CONVERSION_FACTOR = 1;
    public static final double TURN_CONVERSION_FACTOR = 1;

    public static final double DEADBAND = 0.05;

    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.25);

    public static final AngularVelocity MAX_ROTATION = RadiansPerSecond.of(2 * Math.PI);

    public static final double LEFT_STICK_SCAILING = 2;

    public static final double RIGHT_STICK_SCAILING = 3;

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_TURN_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_TURN_ID = 4;
    public static final int BACK_LEFT_DRIVE_ID = 5;
    public static final int BACK_LEFT_TURN_ID = 6;
    public static final int BACK_RIGHT_DRIVE_ID = 7;
    public static final int BACK_RIGHT_TURN_ID = 8;

    public static final int GYRO_ID = 18;

  }
}
