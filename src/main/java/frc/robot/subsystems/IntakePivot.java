package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class IntakePivot extends SubsystemBase {

  @NotLogged private final SparkMax leftMotor;
  @NotLogged private final SparkMax rightMotor;

  @NotLogged private final SparkMaxConfig config;

  @NotLogged private final AbsoluteEncoder leftEncoder;
  @NotLogged private final AbsoluteEncoder rightEncoder;

  @NotLogged private final SparkClosedLoopController leftPid;
  @NotLogged private final SparkClosedLoopController rightPid;

  @NotLogged private final SimpleMotorFeedforward feedforward;

  @NotLogged private final TrapezoidProfile profile;

  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;

  private Angle setpoint;
  private AngularVelocity leftVelocity;
  private Voltage leftVoltage;
  private AngularVelocity rightVelocity;
  private Voltage rightVoltage;
  private Current leftCurrent;
  private Current rightCurrent;
  private Angle leftPosition;
  private Angle rightPosition;

  public IntakePivot() {
    leftMotor = new SparkMax(IntakePivotConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(IntakePivotConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();
    config.closedLoop.p(IntakePivotConstants.PID.kp);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    config.absoluteEncoder.positionConversionFactor(IntakePivotConstants.CONVERSION_FACTOR);
    config.absoluteEncoder.velocityConversionFactor(IntakePivotConstants.CONVERSION_FACTOR / 60.0);

    config.absoluteEncoder.inverted(true);

    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = leftMotor.getAbsoluteEncoder();

    leftPid = leftMotor.getClosedLoopController();

    config.absoluteEncoder.inverted(false);
    config.inverted(true);

    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightEncoder = rightMotor.getAbsoluteEncoder();

    rightPid = rightMotor.getClosedLoopController();

    profile =
        new TrapezoidProfile(
            new Constraints(
                IntakePivotConstants.MAX_VELOCITY.in(RotationsPerSecond),
                IntakePivotConstants.MAX_ACCELRATION.in(RotationsPerSecondPerSecond)));

    feedforward =
        new SimpleMotorFeedforward(
            IntakePivotConstants.FEEDFORWARD.ks, IntakePivotConstants.FEEDFORWARD.kv);

    goal = new TrapezoidProfile.State();
    setpoint = IntakePivotConstants.IN_POSITION;
    motorSetpoint = new TrapezoidProfile.State();
  }

  @Override
  public void periodic() {
    goal = new TrapezoidProfile.State(setpoint.in(Radians), 0);

    motorSetpoint =
        profile.calculate(RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds), motorSetpoint, goal);

    leftPid.setReference(
        motorSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(motorSetpoint.velocity));

    rightPid.setReference(
        motorSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(motorSetpoint.velocity));

    leftPosition = Angle.ofBaseUnits(leftEncoder.getPosition(), Radians);
    leftVelocity = AngularVelocity.ofBaseUnits(leftEncoder.getVelocity(), RadiansPerSecond);

    rightPosition = Angle.ofBaseUnits(rightEncoder.getPosition(), Radians);
    rightVelocity = AngularVelocity.ofBaseUnits(rightEncoder.getVelocity(), RadiansPerSecond);

    leftVoltage =
        Voltage.ofBaseUnits(leftMotor.getBusVoltage() * leftMotor.getAppliedOutput(), Volts);
    rightVoltage =
        Voltage.ofBaseUnits(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage(), Volts);

    leftCurrent = Current.ofBaseUnits(leftMotor.getOutputCurrent(), Amps);
    rightCurrent = Current.ofBaseUnits(rightMotor.getOutputCurrent(), Amps);
  }

  public boolean atSetpoint(Angle setpoint) {
    return leftPosition.gte(setpoint.minus(IntakePivotConstants.TOLERANCE))
        && leftPosition.lte(setpoint.plus(IntakePivotConstants.TOLERANCE))
        && rightPosition.gte(setpoint.minus(IntakePivotConstants.TOLERANCE))
        && rightPosition.lte(setpoint.plus(IntakePivotConstants.TOLERANCE));
  }

  @Override
  public void simulationPeriodic() {
    leftPosition = setpoint;
    rightPosition = setpoint;
  }

  private void setPosition(Angle position) {
    goal = new TrapezoidProfile.State(position.in(Radians), 0);
    setpoint = position;
  }

  public Command setPositionCommand(Angle position) {
    return Commands.runOnce(() -> this.setPosition(position), this);
  }
}
