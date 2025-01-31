package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
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
    public static final Distance L1_SETPOINT = Distance.ofBaseUnits(0.5, Units.Meters);

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD =
        new FeedforwardGains.Builder().kv(0.01).kg(0).build();

    public static final LinearVelocity MAX_VELOCITY =
        LinearVelocity.ofBaseUnits(0.5, Units.MetersPerSecond);
    public static final LinearAcceleration MAX_ACCELRATION =
        LinearAcceleration.ofBaseUnits(0.25, Units.MetersPerSecondPerSecond);

    public static final Distance CONVERSION_FACTOR = Distance.ofBaseUnits(0.05207 * (Math.PI * Math.PI), Meters);

    public static final Current CURRENT_LIMIT = Current.ofBaseUnits(60, Units.Amps);

    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 1;
  }
}
