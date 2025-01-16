package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class Constants {

  public static class RobotConstants {}

  public static class SwerveConstants {
    public static final LinearVelocity MAX_SPEED =
        LinearVelocity.ofBaseUnits(4.25, Units.MetersPerSecond);
    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
        AngularVelocity.ofBaseUnits(0.75, Units.RotationsPerSecond);
    public static final Time SWERVE_UPDATE_PERIOD = Time.ofBaseUnits(20, Units.Millisecond);
  }

  public static class IntakePivotConstants {
    public static final int PIVOT_MOTOR_ID = 9998;
    public static final Angle START_SETPOINT = Angle.ofBaseUnits(0, Units.Degrees);
  }
  //the purpose of this comment is solely to iritate jake >:D
  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 9998;
    public static final int PID_kP = 1;
    public static final int PID_kV = 1;
  }

}
