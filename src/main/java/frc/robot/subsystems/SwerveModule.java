package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase{

  private final SparkFlex driveMotor;
  private final SparkMax turnMotor;
  private final AbsoluteEncoder turnEncoder;
  private final RelativeEncoder driveEncoder;
  private final SparkMaxConfig turnConfig;
  private final SparkFlexConfig driveConfig;
  private final SimpleMotorFeedforward turnFeedforward;
  private final SimpleMotorFeedforward driveFeedforward;
  private Angle turnSetpoint;
  private LinearVelocity driveSetpoint;
  private Angle turnPosition;
  private LinearVelocity driveVelocity;
  private AngularVelocity turnVelocity;
  private Voltage turnVoltage;
  private Voltage driveVoltage;
  private Current turnCurrent;
  private Current driveCurrent;

  public SwerveModule(int driveID, int turnID){

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
    driveConfig.closedLoop.pid(SwerveConstants.DRIVE_PID.kp, SwerveConstants.DRIVE_PID.ki, SwerveConstants.DRIVE_PID.kd);
    driveConfig.smartCurrentLimit((int) SwerveConstants.DRIVE_CURRENT_LIMIT.in(Amps));

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnEncoder = turnMotor.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();

    turnFeedforward = new SimpleMotorFeedforward
    (SwerveConstants.TURN_FEEDFORWARD.ks, SwerveConstants.TURN_FEEDFORWARD.kv, SwerveConstants.TURN_FEEDFORWARD.ka);
    driveFeedforward = new SimpleMotorFeedforward
    (SwerveConstants.TURN_FEEDFORWARD.ks, SwerveConstants.TURN_FEEDFORWARD.kv, SwerveConstants.TURN_FEEDFORWARD.ka);
    
    turnSetpoint = Radians.of(0);
    driveSetpoint = LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond);
    turnPosition = Radians.of(turnEncoder.getPosition());
    driveVelocity = LinearVelocity.ofBaseUnits(driveEncoder.getVelocity(), Units.MetersPerSecond);
    turnVelocity = AngularVelocity.ofBaseUnits(turnEncoder.getVelocity(), Units.RadiansPerSecond);
    driveVoltage = Voltage.ofBaseUnits(driveMotor.getBusVoltage() * driveMotor.getAppliedOutput(), Volts);
    turnVoltage = Voltage.ofBaseUnits(turnMotor.getBusVoltage() * turnMotor.getAppliedOutput(), Volts);
    driveCurrent = Current.ofBaseUnits(driveMotor.getOutputCurrent(), Amps);
    turnCurrent = Current.ofBaseUnits(turnMotor.getOutputCurrent(), Amps);
  }
    
  public void setState(SwerveModuleState swerveModuleState){
    
  } 

  public SwerveModuleState getModuleState(){
    return SwerveModuleState;

  }
}
