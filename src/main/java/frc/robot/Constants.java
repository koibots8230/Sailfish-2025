package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
    public static final int MOTOR_ID = 0;

    public static final AngularVelocity MAX_VELOCITY =
        AngularVelocity.ofBaseUnits(0, Units.RotationsPerSecond);
    public static final AngularAcceleration MAX_ACCELRATION =
        AngularAcceleration.ofBaseUnits(0, Units.RotationsPerSecondPerSecond);

    public static final double POSITION_CONVERSION_FACTOR = 1;
    public static final double VELOCITY_CONVERSION_FACTOR = 1;

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains FEEDFORWARD = new FeedforwardGains.Builder().kv(0).kg(0).build();

    public static final Distance START_SETPOINT = Distance.ofBaseUnits(0, Units.Meters);

    public static final Distance L1_SETPOINT = Distance.ofBaseUnits(1, Units.Meters);
    public static final Distance L2_SETPOINT = Distance.ofBaseUnits(2, Units.Meters);
    public static final Distance L3_SETPOINT = Distance.ofBaseUnits(3, Units.Meters);
    public static final Distance L4_SETPOINT = Distance.ofBaseUnits(4, Units.Meters);
  }
}
