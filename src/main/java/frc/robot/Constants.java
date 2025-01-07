package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.*;

public class Constants {

  public static class RobotConstants {}

  // STEP 6: Add a LinearVelocity MAX_SPEED and a AngularVelocity MAX_ANGULAR_VELOCITY
  public static class SwerveConstants {

    public static final double DEADBAND = 0.05;

    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.25);

    public static final AngularVelocity MAX_ROTATION = RotationsPerSecond.of(Math.PI * 2);

    public static final double LEFT_STICK_SCAILING = 2;

    public static final double RIGHT_STICK_SCAILING = 3;
  }
}
