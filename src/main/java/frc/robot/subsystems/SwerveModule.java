package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

@Logged
public class SwerveModule {
  private final SparkFlex driveMotor;
  private final SparkMax turnMotor;

  private final AbsoluteEncoder turnEncoder;
  private final RelativeEncoder driveEncoder;

  private final SparkMaxConfig turnConfig;
  private final SparkFlexConfig driveConfig;

  private final SimpleMotorFeedforward turnFeedforward;

  private Angle turnSetpoint;
  private LinearVelocity driveSetpoint;

  double drivePosition; //TODO: Put back to measures when fixed
  double turnPosition;
  double driveVelocity;
  private AngularVelocity turnVelocity;

  private Voltage turnVoltage;
  private Voltage driveVoltage;
  private Current turnCurrent;
  private Current driveCurrent;

  private final SparkClosedLoopController turnController;
  private final SparkClosedLoopController driveController;

  private final TrapezoidProfile turnProfile;
  private TrapezoidProfile.State turnGoalState;
  private TrapezoidProfile.State turnSetpointState;

    public SwerveModule(int driveID, int turnID){

      turnProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(SwerveConstants.MAX_TURN_VECLOCITY.in(RadiansPerSecond), SwerveConstants.MAX_TURN_ACCELERATION.in(RadiansPerSecondPerSecond)));

      turnGoalState = new TrapezoidProfile.State(0,0);
      turnSetpointState = new TrapezoidProfile.State(0,0);
  
      turnMotor = new SparkMax(turnID, MotorType.kBrushless);
      driveMotor = new SparkFlex(driveID, MotorType.kBrushless);

      turnConfig = new SparkMaxConfig();

      turnConfig.idleMode(IdleMode.kBrake);

      turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid
        (SwerveConstants.TURN_PID.kp, SwerveConstants.TURN_PID.ki, SwerveConstants.TURN_PID.kd);

      turnConfig.smartCurrentLimit((int) SwerveConstants.TURN_CURRENT_LIMIT.in(Amps));

      turnConfig.absoluteEncoder.positionConversionFactor(SwerveConstants.TURN_CONVERSION_FACTOR);
      turnConfig.absoluteEncoder.velocityConversionFactor(SwerveConstants.TURN_CONVERSION_FACTOR);
  
      driveConfig = new SparkFlexConfig();

      driveConfig.idleMode(IdleMode.kBrake);

      driveConfig.closedLoop.pidf(SwerveConstants.DRIVE_PID.kp, SwerveConstants.DRIVE_PID.ki, SwerveConstants.DRIVE_PID.kd, SwerveConstants.DRIVE_PID.kf);

      driveConfig.smartCurrentLimit((int) SwerveConstants.DRIVE_CURRENT_LIMIT.in(Amps));
  
      turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
      turnEncoder = turnMotor.getAbsoluteEncoder();
      turnController = turnMotor.getClosedLoopController();

      driveEncoder = driveMotor.getEncoder();
      driveController = driveMotor.getClosedLoopController();

    turnFeedforward = new SimpleMotorFeedforward
    (SwerveConstants.TURN_FEEDFORWARD.ks, SwerveConstants.TURN_FEEDFORWARD.kv, SwerveConstants.TURN_FEEDFORWARD.ka);

    turnSetpoint = Radians.of(0);
    driveSetpoint = LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond);
    turnPosition = turnEncoder.getPosition();
    drivePosition = driveEncoder.getPosition();
    driveVelocity = driveEncoder.getVelocity();
    turnVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
    driveVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput(), Volts);
    turnVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput(), Volts);
    driveCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);
    turnCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Amps);
  }
    
  public void setState(SwerveModuleState swerveModuleState){
    turnController.setReference(swerveModuleState.angle.getRadians(), SparkBase.ControlType.kPosition);
    driveController.setReference(swerveModuleState.speedMetersPerSecond, SparkBase.ControlType.kVelocity);

    driveSetpoint = MetersPerSecond.of(swerveModuleState.speedMetersPerSecond);
    turnSetpoint = Radians.of(swerveModuleState.angle.getRadians());
  } 

  public void periodic() {
    driveCurrent =  Current.ofBaseUnits(driveMotor.getOutputCurrent(), Units.Amps);
    turnCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Units.Amps);
    driveVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput(), Volts);
    turnVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput(), Volts);
    drivePosition = driveEncoder.getPosition();
    turnPosition = turnEncoder.getPosition();
    driveVelocity = driveEncoder.getVelocity();
    turnVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);

    turnGoalState = new TrapezoidProfile.State(turnSetpoint.in(Radians), 0);

    turnSetpointState = turnProfile.calculate(RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds), turnSetpointState, turnGoalState);

    turnMotor.getClosedLoopController().setReference(turnSetpointState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, turnFeedforward.calculate(turnSetpointState.velocity));

  }

  public void simulationPeriodic() {
    drivePosition = drivePosition + driveSetpoint.times(RobotConstants.ROBOT_CLOCK_SPEED).in(Meters);
    turnPosition = turnSetpoint.in(Radians);
    driveVelocity = driveSetpoint.in(MetersPerSecond);
  }

  public SwerveModuleState getModuleState(){
    return new SwerveModuleState(driveVelocity, new Rotation2d(turnPosition));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(drivePosition, new Rotation2d(turnPosition));
  }
}
