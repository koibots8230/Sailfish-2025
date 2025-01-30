package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.Thread.State;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

@Logged
public class Swerve extends SubsystemBase{
  @Logged(name = "modules")
  private Pose2d estimatedPosition;
  private Rotation2d simHeading;
  private Rotation2d gyroAngle;
  private SwerveModuleState[] setpointStates;
  private final Pigeon2 gyro;
  private final SwerveModule modules[];
  private final SwerveModuleState[] messuredStates;

  // private final SwerveDrivePoseEstimator odometry;

  public Swerve() {

    // odometry = new SwerveDrivePoseEstimator(Constants.SwerveConstants.KINEMATICS, gyroAngle, this.getModulePostition(), estimatedPosition);

    modules = new SwerveModule[4];

    modules[0] = new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE_ID, SwerveConstants.FRONT_LEFT_TURN_ID);
    modules[1] =  new SwerveModule(SwerveConstants.FRONT_RIGHT_DRIVE_ID, SwerveConstants.FRONT_RIGHT_TURN_ID);
    modules[2] =  new SwerveModule(SwerveConstants.BACK_LEFT_DRIVE_ID, SwerveConstants.BACK_LEFT_TURN_ID);
    modules[3] =  new SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE_ID, SwerveConstants.BACK_RIGHT_TURN_ID);

    

    gyro = new Pigeon2(SwerveConstants.GYRO_ID);

    simHeading = new Rotation2d(0.0);

    estimatedPosition = new Pose2d();

    setpointStates = new SwerveModuleState[4];
    messuredStates = new SwerveModuleState[4];

  }

  @Override
  public void periodic() {
    
    modules[0].periodic();
    modules[1].periodic();
    modules[2].periodic();
    modules[3].periodic();

    gyroAngle = gyro.getRotation2d();

    messuredStates[0] = modules[0].getModuleState();
    messuredStates[1] = modules[1].getModuleState();
    messuredStates[2] = modules[2].getModuleState();
    messuredStates[3] = modules[3].getModuleState();

  //  estimatedPosition = odometry.update(gyroAngle, this.getModulePostition());
  }

  @Override
  public void simulationPeriodic() {
    gyroAngle = simHeading;

    modules[0].simulationPeriodic();
    modules[1].simulationPeriodic();
    modules[2].simulationPeriodic();
    modules[3].simulationPeriodic();
  }

  public SwerveModulePosition[] getModulePostition(){
    return new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    };
  }

 // public Pose2d getOdometryPose(){
   // return odometry.getEstimatedPosition();
 // }

  private void DriveFiledRelativeBlueScailier(double x, double y, double omega){

    double liniarMagintued = Math.pow(Math.hypot(x, y), SwerveConstants.LEFT_STICK_SCAILING);

    Rotation2d directions = new Rotation2d(y, x);

    y = liniarMagintued * -directions.getCos() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);
    x = liniarMagintued * directions.getSin() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);

    omega =  Math.pow(omega, SwerveConstants.RIGHT_STICK_SCAILING) * SwerveConstants.MAX_ROTATION.in(RadiansPerSecond);

    driveFieldRelative(MetersPerSecond.of(MathUtil.applyDeadband(-x, Constants.SwerveConstants.DEADBAND)), MetersPerSecond.of(MathUtil.applyDeadband(y, Constants.SwerveConstants.DEADBAND)), RadiansPerSecond.of(MathUtil.applyDeadband(-omega, Constants.SwerveConstants.DEADBAND)));
  }

  private void DriveFiledRelativeRedScailier(double x, double y, double omega){

    double liniarMagintued = Math.pow(Math.hypot(x, y), SwerveConstants.LEFT_STICK_SCAILING);

    Rotation2d directions = new Rotation2d(y, x);

    y = liniarMagintued * -directions.getCos() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);
    x = liniarMagintued * directions.getSin() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);

    System.out.println("max speed in radians is " + SwerveConstants.MAX_ROTATION.in(RadiansPerSecond));
    System.out.println("rotaton speed is " + Math.pow(omega, SwerveConstants.RIGHT_STICK_SCAILING) * SwerveConstants.MAX_ROTATION.in(RadiansPerSecond));
    omega =  Math.pow(omega, SwerveConstants.RIGHT_STICK_SCAILING) * SwerveConstants.MAX_ROTATION.in(RadiansPerSecond);

    driveFieldRelative(MetersPerSecond.of(MathUtil.applyDeadband(x, Constants.SwerveConstants.DEADBAND)), MetersPerSecond.of(MathUtil.applyDeadband(-y, Constants.SwerveConstants.DEADBAND)), RadiansPerSecond.of(MathUtil.applyDeadband(-omega, Constants.SwerveConstants.DEADBAND)));
  }

  private void driveFieldRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega){
    simHeading = simHeading.plus(new Rotation2d(omega.times(Seconds.of(.02))));
    estimatedPosition = new Pose2d(estimatedPosition.getX() + x.times(RobotConstants.ROBOT_CLOCK_SPEED).in(Meters), estimatedPosition.getY() + (y.times(RobotConstants.ROBOT_CLOCK_SPEED).in(Meters)), new Rotation2d(estimatedPosition.getRotation().getRadians() + omega.times(Seconds.of(.02)).in(Radians)));
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x.times(Seconds.of(.02)).in(Meters), y.times(Seconds.of(.02)).in(Meters), omega.times(Seconds.of(.02)).in(Radians), gyroAngle);
    setpointStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
    
    modules[0].setState(setpointStates[0]);
    modules[1].setState(setpointStates[1]);
    modules[2].setState(setpointStates[2]);
    modules[3].setState(setpointStates[3]);
  }

  public void zeroing(boolean colour){
    System.out.println(colour);
    simHeading = colour == true ?  new Rotation2d(0.0) :  new Rotation2d(Math.PI);
  }
   

   public Command driveFieldRelativeRedCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega){
    return Commands.sequence(
      Commands.run(
() -> DriveFiledRelativeBlueScailier(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble()), this)
);
   }

   public Command driveFieldRelativeBlueCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega){
    return Commands.sequence(
      Commands.run(
() -> DriveFiledRelativeRedScailier(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble()), this)
);
   }

   public Command zeroRobotCommad(boolean colour){
    return Commands.runOnce(() -> zeroing(colour), this);
   }

  /**
   * Step 3b: Public command factory driveFieldRelativeCommand that takes field-relative inputs
   * and returns a command that passes the input parameters into the private driveFieldRelative
   * @param X field-relative X with range of -1.0 to 1.0
   * @param Y field-relative Y with range of -1.0 to 1.0
   * @param Omega field-relative omega with range of -1.0 to 1.0
   */

}
