package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {

  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  private Pose2d estimatedPosition;
  private Rotation2d simHeading;
  private Rotation2d gyroAngle;
  private SwerveModuleState[] setpointStates;
  private final Pigeon2 gyro;

  @Logged
  public class Modules {
    final SwerveModule frontLeft;
    final SwerveModule frontRight;
    final SwerveModule backLeft;
    final SwerveModule backRight;

    public Modules() {
      frontLeft =
          new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE_ID, SwerveConstants.FRONT_LEFT_TURN_ID);
      frontRight =
          new SwerveModule(
              SwerveConstants.FRONT_RIGHT_DRIVE_ID, SwerveConstants.FRONT_RIGHT_TURN_ID);
      backLeft =
          new SwerveModule(SwerveConstants.BACK_LEFT_DRIVE_ID, SwerveConstants.BACK_LEFT_TURN_ID);
      backRight =
          new SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE_ID, SwerveConstants.BACK_RIGHT_TURN_ID);
    }
  }

  private final Modules modules;

  private final SwerveModuleState[] messuredStates;

  private final SwerveDrivePoseEstimator odometry;

  private boolean isBlue;

  RobotConfig config;

  public Swerve() {

    this.isBlue = true;

    modules = new Modules();

    estimatedPosition = new Pose2d();

    gyro = new Pigeon2(SwerveConstants.GYRO_ID);

    gyroAngle = new Rotation2d();

    odometry =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.KINEMATICS,
            gyroAngle,
            this.getModulePostition(),
            estimatedPosition);

    simHeading = new Rotation2d(0.0);

    setpointStates = new SwerveModuleState[4];
    messuredStates = new SwerveModuleState[4];

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getEstimatedPosition,
        this::setOdometry,
        this::getChassisSpeeds,
        this::driveRobotRelative,
        SwerveConstants.pathPlannerFF,
        config,
        () -> getIsBlue(),
        this);
  }

  public boolean getIsBlue() {
    return isBlue;
  }

  public void followTerjectory(SwerveSample sample) {
    Pose2d pose = getEstimatedPosition();

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

//    driveRobotRelative(speeds);
  }

  public void setOdometry(Pose2d pose) {
    odometry.resetPose(pose);
  }

  public void setIsBlue(boolean colour) {
    isBlue = colour;
  }

  @Override
  public void periodic() {

    modules.frontLeft.periodic();
    modules.frontRight.periodic();
    modules.backLeft.periodic();
    modules.backRight.periodic();

    estimatedPosition =
        odometry.update(
            !isBlue ? gyroAngle : gyroAngle.minus(new Rotation2d(Math.PI)),
            this.getModulePostition());

    gyroAngle = gyro.getRotation2d();
    messuredStates[0] = modules.frontLeft.getModuleState();
    messuredStates[1] = modules.frontRight.getModuleState();
    messuredStates[2] = modules.backLeft.getModuleState();
    messuredStates[3] = modules.backRight.getModuleState();
  }

  @Override
  public void simulationPeriodic() {
    simHeading =
        simHeading.plus(
            new Rotation2d(
                getChassisSpeeds().omegaRadiansPerSecond
                    * RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds)));
    gyroAngle = simHeading;

    modules.frontLeft.simulationPeriodic();
    modules.frontRight.simulationPeriodic();
    modules.backLeft.simulationPeriodic();
    modules.backRight.simulationPeriodic();
  }

  public SwerveModulePosition[] getModulePostition() {
    return new SwerveModulePosition[] {
      modules.frontLeft.getPosition(),
      modules.frontRight.getPosition(),
      modules.backLeft.getPosition(),
      modules.backRight.getPosition()
    };
  }

  private void driveFieldRelativeScaler(double x, double y, double omega) {

    double linearMagnitude = Math.pow(Math.hypot(x, y), SwerveConstants.LEFT_STICK_SCAILING);

    Rotation2d direction = new Rotation2d(y, x);

    y = linearMagnitude * -direction.getCos() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);
    x = linearMagnitude * direction.getSin() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);

    omega =
        Math.pow(omega, SwerveConstants.RIGHT_STICK_SCAILING)
            * SwerveConstants.MAX_ROTATION.in(RadiansPerSecond);

    driveFieldRelative(
        MetersPerSecond.of(MathUtil.applyDeadband(-x, Constants.SwerveConstants.DEADBAND)),
        MetersPerSecond.of(MathUtil.applyDeadband(y, Constants.SwerveConstants.DEADBAND)),
        RadiansPerSecond.of(MathUtil.applyDeadband(-omega, Constants.SwerveConstants.DEADBAND)));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.KINEMATICS.toChassisSpeeds(messuredStates);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setpointStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, Constants.SwerveConstants.MAX_SPEED);

    modules.frontLeft.setState(setpointStates[0]);
    modules.frontRight.setState(setpointStates[1]);
    modules.backLeft.setState(setpointStates[2]);
    modules.backRight.setState(setpointStates[3]);
  }

  private void driveFieldRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x.in(MetersPerSecond), y.in(MetersPerSecond), omega.in(RadiansPerSecond), gyroAngle);
        driveRobotRelative(speeds);
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Pose2d getEstimatedPosition() {
    return estimatedPosition;
  }

  public Command driveFieldRelativeCommand(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () -> driveFieldRelativeScaler(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble()),
        this);
  }

  public Command zeroGyroCommand(boolean colour) {
    return Commands.runOnce(() -> zeroGyro(), this);
  }

  /**
   * Step 3b: Public command factory driveFieldRelativeCommand that takes field-relative inputs and
   * returns a command that passes the input parameters into the private driveFieldRelative
   *
   * @param X field-relative X with range of -1.0 to 1.0
   * @param Y field-relative Y with range of -1.0 to 1.0
   * @param Omega field-relative omega with range of -1.0 to 1.0
   */
}
