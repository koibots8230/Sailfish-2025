package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

@Logged
public class Swerve extends SubsystemBase{

  private Pose2d estimatedPosition;


  public Swerve() {

    estimatedPosition = new Pose2d();

  }

  private void DriveFiledRelativeScailier(double x, double y, double omega){


    double liniarMagintued = Math.pow(Math.hypot(x, y), SwerveConstants.LEFT_STICK_SCAILING);

    Rotation2d directions = new Rotation2d(y, x);

    y = liniarMagintued * -directions.getCos() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);
    x = liniarMagintued * directions.getSin() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);

    System.out.println("max speed in radians is " + SwerveConstants.MAX_ROTATION.in(RadiansPerSecond));
    System.out.println("rotaton speed is " + Math.pow(omega, SwerveConstants.RIGHT_STICK_SCAILING) * SwerveConstants.MAX_ROTATION.in(RadiansPerSecond));
    omega =  Math.pow(omega, SwerveConstants.RIGHT_STICK_SCAILING) * SwerveConstants.MAX_ROTATION.in(RadiansPerSecond);

    driveFieldRelative(MetersPerSecond.of(MathUtil.applyDeadband(-x, Constants.SwerveConstants.DEADBAND)), MetersPerSecond.of(MathUtil.applyDeadband(y, Constants.SwerveConstants.DEADBAND)), DegreesPerSecond.of(MathUtil.applyDeadband(-omega, Constants.SwerveConstants.DEADBAND)));
  }


  private void driveFieldRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega){

    estimatedPosition = estimatedPosition.transformBy(new Transform2d(x.times(Seconds.of(.02)).in(Meters), y.times(Seconds.of(.02)).in(Meters), new Rotation2d(omega.times(Seconds.of(.02)).in(Degrees))));

  }
    
      /**
   * Step 3a: Private Method driveFieldRelative that takes field-relative inputs
   *
   * @param X field-relative X with range of -1.0 to 1.0
   * @param Y field-relative Y with range of -1.0 to 1.0
   * @param Omega field-relative omega with range of -1.0 to 1.0
   */

   public Command driveFieldRelativeCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega){
    return Commands.sequence(
      Commands.run(
() -> DriveFiledRelativeScailier(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble()), this)
);
   }

  /**
   * Step 3b: Public command factory driveFieldRelativeCommand that takes field-relative inputs
   * and returns a command that passes the input parameters into the private driveFieldRelative
   * @param X field-relative X with range of -1.0 to 1.0
   * @param Y field-relative Y with range of -1.0 to 1.0
   * @param Omega field-relative omega with range of -1.0 to 1.0
   */

  // STEP 5: In the driveFieldRelative update the estimatedPose by scaling the input
  // parameters and adding them to the current pose values.

  // STEP 6: Use the MAX speeds constants to scale the field-relative inputs to update the
  // estimatedPose

}
