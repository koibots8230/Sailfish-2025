package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
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
    public static final Distance INTAKE_SETPOINT = Distance.ofBaseUnits(0.005, Units.Meters);
    public static final Distance L2_SETPOINT =
        Distance.ofBaseUnits(1.18, Units.Meters); // untested value
    public static final Distance L3_SETPOINT =
        Distance.ofBaseUnits(2.04, Units.Meters); //  untested value

    public static final PIDGains PID = new PIDGains.Builder().kp(3.3).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(2.05).kg(0).build();

    public static final LinearVelocity MAX_VELOCITY =
        LinearVelocity.ofBaseUnits(24, Units.MetersPerSecond);
    public static final LinearAcceleration MAX_ACCELRATION =
        LinearAcceleration.ofBaseUnits(6, Units.MetersPerSecondPerSecond);

    public static final Distance CONVERSION_FACTOR =
        Distance.ofBaseUnits((0.05207 * Math.PI) * 2, Meters);

    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(60, Units.Amps);

    public static final int MAIN_MOTOR_ID = 11;
    public static final int SECONDARY_MOTOR_ID = 12;
    public static final int HALL_EFFECTS_SENSOR = 0;
  }

  public static class EndEffectorConstants {
    public static final double INTAKE_SPEED = 750;
    public static final double OUTTAKE_SPEED =
        1000; // TODO: Turn back into units once not bugged anymore :(

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.0001).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0.0002).build();

    public static final Distance TRIGGER_DISTANCE = Distance.ofBaseUnits(85, Units.Millimeters);

    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(40, Units.Amps);

    public static final int MOTOR_ID = 22;
    public static final int LASERCAN_ID = 21;
  }

  public static class EffectorPivotConsants {

    public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(4 * Math.PI);
    public static final AngularAcceleration MAX_ACCELERATION =
        RadiansPerSecondPerSecond.of(3 * Math.PI);

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0).kg(0).build();

    public static final double CONVERSION_FACTOR = 2 * Math.PI;

    public static final Current CURRENT_LIMIT = Amps.of(50);

    public static final int MOTOR_ID = 30;
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

    public static final AngularVelocity MAX_TURN_VECLOCITY = RadiansPerSecond.of(10 * Math.PI);
    public static final AngularAcceleration MAX_TURN_ACCELERATION =
        RadiansPerSecondPerSecond.of(16 * Math.PI);

    public static final PIDGains TURN_PID = new PIDGains.Builder().kp(3).kd(0.0).build();
    public static final PIDGains DRIVE_PID = new PIDGains.Builder().kp(0.37).build();
    public static final FeedforwardGains TURN_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.55).build();
    public static final FeedforwardGains DRIVE_FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.24).build();

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

    public static final double SWERVE_GEARING = 5.50;

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

    public static final int GYRO_ID = 9;
  }

  public static class AlignConstants {
    public static final Distance MIN_DISTANCE = Meters.of(2.0);

    public static final Pose2d[] REEF_SIDES =
        new Pose2d[] { // Starts from side closest to DS and goes counterclockwise
          new Pose2d(3.23215, 4.02082, Rotation2d.fromDegrees(180)),
          new Pose2d(3.861181, 2.93278749196, Rotation2d.fromDegrees(240)),
          new Pose2d(5.117465, 2.93278749196, Rotation2d.fromDegrees(300)),
          new Pose2d(5.746496, 4.02082, Rotation2d.fromDegrees(0)),
          new Pose2d(5.117465, 5.10885250804, Rotation2d.fromDegrees(60)),
          new Pose2d(3.861181, 5.10885250804, Rotation2d.fromDegrees(120))
        };

    public static final Distance RED_REEF_OFFSET = Meters.of(8.569706);

    public static final PIDGains TRANSLATE_PID = new PIDGains.Builder().kp(2).build();
    public static final PIDGains ANGLE_PID = new PIDGains.Builder().kp(2).build();

    public static final Angle DIRECTION_ANGLE_RANGE_CLOSE = Radians.of(Math.PI / 1.85);

    public static final double DISTANCE_ANGLE_RANGE_SCALAR = 0.85;

    public static final Distance POLE_SPACING = Meters.of(0.1651);

    public static final Distance EFFECTOR_OFFSET = Meters.of(0.0216154);
  }

  public static class IntakePivotConstants {
    public static final Angle OUT_POSITION = Angle.ofBaseUnits(10, Radians);
    public static final Angle IN_POSITION = Angle.ofBaseUnits(0, Radians);
    public static final AngularVelocity MAX_VELOCITY =
        AngularVelocity.ofBaseUnits(0, Units.RotationsPerSecond);
    public static final AngularAcceleration MAX_ACCELRATION =
        AngularAcceleration.ofBaseUnits(0, Units.RotationsPerSecondPerSecond);

    public static final Angle TOLERANCE =
        Radians.of(0.025); // TODO CHANGE THE PLUS AND MINUS VALUES HERE TO SOMTHING RELEVENT

    public static final double POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;
    public static final double VELCOITY_CONVERSION_FACTOR = (2.0 * Math.PI) / 60.0;

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0).kg(0).build();

    public static final int LEFT_MOTOR_ID = 31;
    public static final int RIGHT_MOTOR_ID = 32;
    public static final int LIMIT_SWITCH_CHANEL = 1;
  }

  public static class IntakeConstants {
    public static final AngularVelocity INTAKE_VELOCITY = AngularVelocity.ofBaseUnits(2500, RPM);
    public static final AngularVelocity REVERSE_INTAKE_VELOCITY =
        AngularVelocity.ofBaseUnits(-2000, RPM);

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.00002).build();

    public static final Current CURRENT_LIMIT = Amps.of(80);

    public static final int MOTOR_ID = 33;
  }

  public static class IndexerConstants {
    public static final double INDEX_VELOCITY = 2000;
    public static final double REVERSE_VELOCITY = -1500;

    public static final PIDGains PID = new PIDGains.Builder().kp(0.0000).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.00023).kg(0).build();

    public static final int MOTOR_ID = 41;
  }

  public static class VisionConstants {
    public static final int ACTIVE_CAMERAS = 2;

    public static final Pose2d[] CAMERA_POSITIONS = {
      new Pose2d(-13.5 + 0.952, 6.0, Rotation2d.fromDegrees(180)),
      new Pose2d(-13.5 + 0.952, -6.5, Rotation2d.fromDegrees(180))
      // new Pose2d(-0.0, 0.0, Rotation2d.fromDegrees(180)),
      // new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))
    }; // x is forward, y is left, counterclockwise on rotation

    public static final String[][] TOPIC_NAMES = {
      {"camera0/tvec", "camera0/rmat", "camera0/id"},
      {"camera1/tvec", "camera1/rmat", "camera1/id"}
      // {"Cam3Tvec", "Cam3Rvec", "Cam3Ids"},
      // {"Cam4Tvec", "Cam4Rvec", "Cam4Ids"}
    };

    public static final double[] VECTOR_DEFAULT_VALUE = {0};
    public static final int ID_DEFAULT_VALUE = 0;

    public static final Distance MAX_MEASUREMENT_DIFFERENCE = Meters.of(1.5);
    public static final Rotation2d MAX_ANGLE_DIFFERENCE = Rotation2d.fromDegrees(10);

    public static final double ROTATION_STDEV = 50 * Math.PI;
    public static final double TRANSLATION_STDEV_ORDER = 2;
    public static final double TRANSLATION_STDEV_SCALAR = 2;
  }
}
