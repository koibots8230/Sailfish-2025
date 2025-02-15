package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;
import frc.lib.util.Wheel;

public class Constants {

  public static class ElevatorConstants {
    public static final Distance START_SETPOINT = Distance.ofBaseUnits(0.005, Units.Meters);
    public static final Distance L1_SETPOINT = Distance.ofBaseUnits(2.65, Units.Meters);

    public static final PIDGains PID = new PIDGains.Builder().kp(2.2).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(4.9).kg(0.37).build();

    public static final LinearVelocity MAX_VELOCITY =
        LinearVelocity.ofBaseUnits(24, Units.MetersPerSecond);
    public static final LinearAcceleration MAX_ACCELRATION =
        LinearAcceleration.ofBaseUnits(12, Units.MetersPerSecondPerSecond);

    public static final Distance CONVERSION_FACTOR =
        Distance.ofBaseUnits((0.05207 * Math.PI) * 2, Meters);

    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(60, Units.Amps);

    public static final int MAIN_MOTOR_ID = 31;
    public static final int SECONDARY_MOTOR_ID = 30;
    public static final int HALL_EFFECTS_SENSOR = 0;
  }

  public static class EndEffectorConstants {
    public static final double INTAKE_SPEED = 250;
    public static final double OUTTAKE_SPEED =
        1000; // TODO: Turn back into units once not bugged anymore :(

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.0001).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0.0002).build();

    public static final Distance TRIGGER_DISTANCE = Distance.ofBaseUnits(0.085, Units.Meters);

    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(40, Units.Amps);

    public static final int MOTOR_ID = 10;
    public static final int LASERCAN_ID = 34;
  }

  public static class RobotConstants {

    public static final Distance WIDTH = Inches.of(23.5);
    public static final Distance LENGTH = Inches.of(23.5);

    public static final Time ROBOT_CLOCK_SPEED = Time.ofBaseUnits(0.02, Units.Seconds);
  }

  public static class SwerveConstants {

    // to do make a accleration cap

    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.25);
    public static final AngularVelocity MAX_ROTATION = RadiansPerSecond.of(2 * Math.PI);

    public static final AngularVelocity MAX_TURN_VECLOCITY = RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration MAX_TURN_ACCELERATION =
        RadiansPerSecondPerSecond.of(4 * Math.PI);

    public static final PIDGains TURN_PID = new PIDGains.Builder().kp(1).kd(0.0).build();
    public static final PIDGains DRIVE_PID = new PIDGains.Builder().kp(0.07).build();
    public static final FeedforwardGains TURN_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.55).build();
    public static final FeedforwardGains DRIVE_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.21).build();

    public static final double MAX_VELOCITY = 10 * Math.PI;
    public static final double MAX_ACCELRATION = 16 * Math.PI;

    public static final PPHolonomicDriveController pathPlannerFF =
        new PPHolonomicDriveController(new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0));

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(RobotConstants.LENGTH.divide(2), RobotConstants.WIDTH.divide(2)),
            new Translation2d(RobotConstants.LENGTH.divide(2), RobotConstants.WIDTH.divide(-2)),
            new Translation2d(RobotConstants.LENGTH.divide(-2), RobotConstants.WIDTH.divide(2)),
            new Translation2d(RobotConstants.LENGTH.divide(-2), RobotConstants.WIDTH.divide(-2)));

    public static final Wheel SWERVE_WHEEL = new Wheel(Inches.of(1.5));

    public static final double SWERVE_GEARING = 5.08;

    public static final double DRIVE_CONVERSION_FACTOR = (0.038 * 2 * Math.PI) / SWERVE_GEARING;
    public static final double TURN_CONVERSION_FACTOR = 2 * Math.PI;

    public static final Rotation2d[] OFFSETS = {
      Rotation2d.fromRadians((3 * Math.PI) / 2.0),
      Rotation2d.fromRadians(0),
      Rotation2d.fromRadians(Math.PI),
      Rotation2d.fromRadians(Math.PI / 2.0)
    };

    public static final Current TURN_CURRENT_LIMIT = Current.ofBaseUnits(30, Units.Amps);
    public static final Current DRIVE_CURRENT_LIMIT = Current.ofBaseUnits(80, Units.Amps);

    public static final double DEADBAND = 0.05;

    public static final double LEFT_STICK_SCAILING = 2;

    public static final double RIGHT_STICK_SCAILING = 1;

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

  public static class IntakePivotConstants {
    public static final Angle OUT_POSITION = Angle.ofBaseUnits(10, Radians);
    public static final Angle IN_POSITION = Angle.ofBaseUnits(0, Radians);
    public static final AngularVelocity MAX_VELOCITY =
        AngularVelocity.ofBaseUnits(0, Units.RotationsPerSecond);
    public static final AngularAcceleration MAX_ACCELRATION =
        AngularAcceleration.ofBaseUnits(0, Units.RotationsPerSecondPerSecond);

    public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
    public static final double VELCOITY_CONVERSION_FACTOR = (2.0 * Math.PI) / 60.0;

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0).kg(0).build();

    public static final int INTAKE_PIVOT_MOTOR_ID = 12;
    public static final int INTAKE_PIVOT_SWITCH_CHANEL = 1;
  }

  public static class IntakeConstants {
    public static final AngularVelocity INTAKE_VELOCITY = AngularVelocity.ofBaseUnits(10, RPM);
    public static final AngularVelocity REVERSE_INTAKE_VELOCITY =
        AngularVelocity.ofBaseUnits(10, RPM);

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0).kg(0).build();

    public static final int INTAKE_LEFT_MOTOR_ID = 14;
    public static final int INTAKE_RIGHT_MOTOR_ID = 11;
  }

  public static class IndexerConstants {
    public static final AngularVelocity INDEX_VELOCITY = AngularVelocity.ofBaseUnits(10, RPM);

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0).kg(0).build();

    public static final int MOTOR_ID = 13;
  }
}
