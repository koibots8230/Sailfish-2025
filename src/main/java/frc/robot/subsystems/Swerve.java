package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ReefAlignState;
import frc.lib.util.VisionMeasurement;
import frc.robot.Constants.*;
import java.util.function.DoubleSupplier;

@Logged
public class Swerve extends SubsystemBase {
  private Pose2d estimatedPosition;
  private Pose2d trajectoryToFollow = new Pose2d();
  private Rotation2d simHeading;
  private Rotation2d gyroAngle;
  private SwerveModuleState[] setpointStates;
  private final Pigeon2 gyro;

  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

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

  private ReefAlignState reefAlignState;

  private Pose2d alignTarget;

  private final PIDController anglePID;

  RobotConfig config;

  public Swerve() {

    this.isBlue = true;

    modules = new Modules();

    estimatedPosition = new Pose2d();

    gyro = new Pigeon2(SwerveConstants.GYRO_ID);

    gyroAngle = new Rotation2d();

    odometry =
        new SwerveDrivePoseEstimator(
            SwerveConstants.KINEMATICS, gyroAngle, this.getModulePostition(), estimatedPosition);

    simHeading = new Rotation2d(0.0);

    setpointStates = new SwerveModuleState[4];
    messuredStates = new SwerveModuleState[4];

    // try{
    //   config = RobotConfig.fromGUISettings();
    // }
    // catch (Exception e){
    //   e.printStackTrace();
    // }

    // AutoBuilder.configure(this::getEstimatedPosition, this::setOdometry, this::getChassisSpeeds,
    // this::driveRobotRelative, SwerveConstants.pathPlannerFF, config, () -> setColour(), this);

    anglePID =
        new PIDController(
            AlignConstants.ANGLE_PID.kp, AlignConstants.ANGLE_PID.ki, AlignConstants.ANGLE_PID.kd);

    anglePID.enableContinuousInput(-Math.PI, Math.PI);

    reefAlignState = ReefAlignState.disabled;
    alignTarget = new Pose2d();

    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public boolean getIsBlue() {
    return isBlue;
  }

  public void setOdometry(Pose2d pose) {
    System.out.println("Resetting Odometry: " + pose);
    simHeading = pose.getRotation();
    odometry.resetPose(pose);
  }

  public void setIsBlue(boolean colour) {
    isBlue = colour;
  }

  private void setReefAlignState(ReefAlignState state) {
    reefAlignState = state;
  }

  @Override
  public void periodic() {

    modules.frontLeft.periodic();
    modules.frontRight.periodic();
    modules.backLeft.periodic();
    modules.backRight.periodic();

    estimatedPosition =
        odometry.update(
            // gyroAngle,
            isBlue ? gyroAngle : gyroAngle.minus(new Rotation2d(Math.PI)),
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

  private Pose2d getAssistVelocity(
      Translation2d targetPose, Rotation2d targetAngle, double xInput, double yInput) {
    Translation2d[] points =
        new Translation2d[] {
          this.getEstimatedPosition().getTranslation(),
          new Translation2d(
              this.getEstimatedPosition().getX() + xInput,
              this.getEstimatedPosition().getY() + yInput)
        };

    Rotation2d angleToTarget =
        Rotation2d.fromRadians(
            Math.atan2(
                this.getEstimatedPosition().getY() - targetPose.getY(),
                this.getEstimatedPosition().getX() - targetPose.getX()));

    Distance distancePerpToVel =
        Meters.of( // Looks complicated, but just the "Line from two points" from this
            // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
            Math.abs(
                    ((points[1].getY() - points[0].getY()) * targetPose.getX())
                        - ((points[1].getX() - points[0].getX()) * targetPose.getY())
                        + (points[1].getX() * points[0].getY())
                        - (points[1].getY() * points[0].getX()))
                / Math.sqrt(
                    Math.pow((points[1].getY() - points[0].getY()), 2)
                        + Math.pow((points[1].getX() - points[0].getX()), 2)));

    LinearVelocity assistVelocity =
        MetersPerSecond.of(distancePerpToVel.in(Meters) * AlignConstants.TRANSLATE_PID.kp);

    return new Pose2d(
        assistVelocity.in(MetersPerSecond) * angleToTarget.getCos(),
        assistVelocity.in(MetersPerSecond) * angleToTarget.getSin(),
        new Rotation2d(
            anglePID.calculate(
                gyroAngle.getRadians(), targetAngle.getRadians() + (isBlue ? 0 : Math.PI))));
  }

  private Distance distanceToPose(Pose2d pose) {
    return Meters.of(
        Math.hypot(
            pose.getX() - this.getEstimatedPosition().getX(),
            pose.getY() - this.getEstimatedPosition().getY()));
  }

  private Pose2d getReefSide() {
    Pose2d closest = new Pose2d(999, 999, Rotation2d.kZero);
    for (Pose2d side : AlignConstants.REEF_SIDES) {
      side =
          isBlue
              ? side
              : new Pose2d(
                  side.getX() + AlignConstants.RED_REEF_OFFSET.in(Meters),
                  side.getY(),
                  side.getRotation());
      closest = distanceToPose(closest).lt(distanceToPose(side)) ? closest : side;
    }
    return closest;
  }

  private Translation2d getEffectorOffset(Pose2d side) {
    return new Translation2d(
        (Math.sin(2 * side.getRotation().getRadians())
            * AlignConstants.EFFECTOR_OFFSET.in(Meters)
            * (side.getRotation().getRadians() >= Math.PI / 3.0
                    && side.getRotation().getRadians() <= (2 * Math.PI) / 3.0
                ? -1
                : 1)),
        (Math.cos(2 * side.getRotation().getRadians())
            * AlignConstants.EFFECTOR_OFFSET.in(Meters)
            * (side.getRotation().getRadians() >= Math.PI / 3.0
                    && side.getRotation().getRadians() <= (2 * Math.PI) / 3.0
                ? -1
                : 1)));
  }

  private Translation2d getPoleTranslation(Pose2d side, boolean rightPole) {
    return new Translation2d(
        side.getX()
            + (Math.sin(2 * side.getRotation().getRadians())
                * AlignConstants.POLE_SPACING.in(Meters)
                * (rightPole ? 1 : -1)
                * (side.getRotation().getRadians() == Math.PI ? -1 : 1)),
        side.getY()
            + (Math.cos(2 * side.getRotation().getRadians())
                * AlignConstants.POLE_SPACING.in(Meters)
                * (rightPole ? 1 : -1)
                * (side.getRotation().getRadians() == Math.PI ? -1 : 1)));
  }

  private Pose2d reefAlignAssist(double xInput, double yInput, double omega) {
    xInput = isBlue ? xInput : -xInput;
    yInput = isBlue ? yInput : -yInput;
    omega = isBlue ? omega : -omega;

    Pose2d closestSide = getReefSide();

    Pose2d pose;
    switch (reefAlignState) {
      case rightSide:
        pose =
            new Pose2d(
                getPoleTranslation(closestSide, true).plus(getEffectorOffset(closestSide)),
                closestSide.getRotation());
        break;
      case leftSide:
        pose =
            new Pose2d(
                getPoleTranslation(closestSide, false).plus(getEffectorOffset(closestSide)),
                closestSide.getRotation());
        break;
      case disabled:
        alignTarget = Pose2d.kZero;
        return Pose2d.kZero;
      default:
        alignTarget = Pose2d.kZero;
        return Pose2d.kZero;
    }

    if (distanceToPose(pose).gte(AlignConstants.MIN_DISTANCE)) {
      alignTarget = Pose2d.kZero;
      return Pose2d.kZero;
    }

    Rotation2d movementDirection = Rotation2d.fromRadians(Math.atan2(yInput, xInput));

    Rotation2d angleRange =
        Rotation2d.fromRadians(
            AlignConstants.DIRECTION_ANGLE_RANGE_CLOSE.in(Radians)
                - (distanceToPose(pose).in(Meters) * AlignConstants.DISTANCE_ANGLE_RANGE_SCALAR));

    if (Math.abs(
            (movementDirection.getRadians()
                        - pose.getRotation().unaryMinus().getRadians()
                        + Math.PI)
                    % (2 * Math.PI)
                - Math.PI)
        > angleRange.getRadians()) {
      alignTarget = Pose2d.kZero;
      return Pose2d.kZero;
    }

    alignTarget = pose;

    return getAssistVelocity(pose.getTranslation(), pose.getRotation(), xInput, yInput);
  }

  private void driveFieldRelativeScaler(double x, double y, double omega) {
    double linearMagnitude = Math.pow(Math.hypot(x, y), SwerveConstants.LEFT_STICK_SCAILING);

    Rotation2d direction = new Rotation2d(y, x);

    y = linearMagnitude * -direction.getCos() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);
    x = linearMagnitude * -direction.getSin() * SwerveConstants.MAX_SPEED.in(MetersPerSecond);

    omega =
        Math.pow(omega, SwerveConstants.RIGHT_STICK_SCAILING)
            * SwerveConstants.MAX_ROTATION.in(RadiansPerSecond);

    Pose2d assist = reefAlignAssist(-x, y, omega);

    driveFieldRelative(
        MetersPerSecond.of(
            MathUtil.applyDeadband(
                x + (assist.getX() * (isBlue ? -1 : 1)), SwerveConstants.DEADBAND)),
        MetersPerSecond.of(
            MathUtil.applyDeadband(
                y + (assist.getY() * (isBlue ? -1 : 1)), SwerveConstants.DEADBAND)),
        RadiansPerSecond.of(
            MathUtil.applyDeadband(
                -omega + assist.getRotation().getRadians(), SwerveConstants.DEADBAND)));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.KINEMATICS.toChassisSpeeds(messuredStates);
  }

  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    setpointStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_SPEED);

    modules.frontLeft.setState(setpointStates[0]);
    modules.frontRight.setState(setpointStates[1]);
    modules.backLeft.setState(setpointStates[2]);
    modules.backRight.setState(setpointStates[3]);
  }

  private void driveFieldRelative(LinearVelocity x, LinearVelocity y, AngularVelocity omega) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x.in(MetersPerSecond), y.in(MetersPerSecond), omega.in(RadiansPerSecond), gyroAngle);

    driveFieldRelative(speeds);
  }

  private void driveFieldRelative(ChassisSpeeds speeds) {
    driveRobotRelative(
        speeds,
        new DriveFeedforwards(
            new double[] {0.0},
            new double[] {0.0},
            new double[] {0.0},
            new double[] {0.0},
            new double[] {0.0}));
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Pose2d getEstimatedPosition() {
    return odometry.getEstimatedPosition();
  }

  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  public void addVisionMeasurement(VisionMeasurement measurement) {
    odometry.addVisionMeasurement(
        measurement.pose,
        measurement.timestamp,
        VecBuilder.fill(
            measurement.translationStdev, measurement.translationStdev, measurement.rotationStdev));
  }

  public void followTrajectory(SwerveSample sample) {
    trajectoryToFollow = sample.getPose();
    System.out.println("Traj: " + trajectoryToFollow);
    // Get the current pose of the robot
    Pose2d pose = odometry.getEstimatedPosition();
    System.out.println("Pose: " + pose);

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

    // Apply the generated speeds
    System.out.println("Spd: " + speeds);
    driveFieldRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation()));
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

  public Command setReefAlignStateCommand(ReefAlignState state) {
    return Commands.runOnce(() -> this.setReefAlignState(state));
  }
}
