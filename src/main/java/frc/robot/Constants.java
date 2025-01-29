package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

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
    public static final Time ROBOT_CLOCK_SPEED = Time.ofBaseUnits(20, Units.Milliseconds);
  }

  public static class SwerveConstants {
    public static final LinearVelocity MAX_SPEED =
        LinearVelocity.ofBaseUnits(4.25, Units.MetersPerSecond);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
        AngularVelocity.ofBaseUnits(0.75, Units.RotationsPerSecond);
    public static final Time SWERVE_UPDATE_PERIOD = Time.ofBaseUnits(20, Units.Millisecond);
  }

  public static class ElevatorConstants {
    public static final Distance START_SETPOINT = Distance.ofBaseUnits(0, Units.Meters);
    public static final Distance L1_SETPOINT = Distance.ofBaseUnits(1, Units.Meters);
    public static final Distance L2_SETPOINT = Distance.ofBaseUnits(2, Units.Meters);
    public static final Distance L3_SETPOINT = Distance.ofBaseUnits(3, Units.Meters);
    public static final Distance L4_SETPOINT = Distance.ofBaseUnits(4, Units.Meters);

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0).kg(0).build();

    public static final AngularVelocity MAX_VELOCITY =
        AngularVelocity.ofBaseUnits(0, Units.RotationsPerSecond);
    public static final AngularAcceleration MAX_ACCELRATION =
        AngularAcceleration.ofBaseUnits(0, Units.RotationsPerSecondPerSecond);

    public static final double CONVERSION_FACTOR = 1;

    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(60, Units.Amps);

    public static final int MOTOR_ID = 0;
  }

  public static class EndEffectorConstants {
    public static final AngularVelocity INTAKE_SPEED = AngularVelocity.ofBaseUnits(3000, Units.RPM);
    public static final AngularVelocity OUTTAKE_SPEED = AngularVelocity.ofBaseUnits(3000, Units.RPM);

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS = new FeedforwardGains.Builder().kv(0).build();

    public static final double CONVERSION_FACTOR = 1;

    public static final Distance TRIGGER_DISTANCE = Distance.ofBaseUnits(1, Units.Millimeters);

    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(40, Units.Amps);

    public static final int MOTOR_ID = 98;
    public static final int LASERCAN_ID = 99;
  }
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
