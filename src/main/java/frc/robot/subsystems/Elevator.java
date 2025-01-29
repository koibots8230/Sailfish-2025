package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Elevator extends SubsystemBase {

  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final TrapezoidProfile profile;
  private final AbsoluteEncoder motorEncoder;
  private final ElevatorFeedforward feedforward;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;
  private Distance setpoint;
  private Distance position;
  private LinearVelocity velocity;
  private Voltage voltage;
  private Current current;

  public Elevator() {
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY.in(RotationsPerSecond),
                ElevatorConstants.MAX_ACCELRATION.in(RotationsPerSecondPerSecond)));

    motor = new SparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ElevatorConstants.PID.kp, ElevatorConstants.PID.ki, ElevatorConstants.PID.kd);
    motorConfig.smartCurrentLimit((int) ElevatorConstants.CURRENT_LIMIT.in(Amps));
    motorConfig.absoluteEncoder.positionConversionFactor(ElevatorConstants.CONVERSION_FACTOR);
    motorConfig.absoluteEncoder.velocityConversionFactor(ElevatorConstants.CONVERSION_FACTOR);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorEncoder = motor.getAbsoluteEncoder();

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.FEEDFORWARD.ks,
            ElevatorConstants.FEEDFORWARD.kg,
            ElevatorConstants.FEEDFORWARD.kv,
            ElevatorConstants.FEEDFORWARD.ka);

    goal = new TrapezoidProfile.State();
    setpoint = ElevatorConstants.START_SETPOINT;
    motorSetpoint = new TrapezoidProfile.State();

    position = Distance.ofBaseUnits(motorEncoder.getPosition(), Meters);
    velocity = LinearVelocity.ofBaseUnits(motorEncoder.getVelocity(), MetersPerSecond);
    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
  }

  @Override
  public void periodic() {
    goal =
        new TrapezoidProfile.State(
            setpoint.in(Meters), RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds));

    motorSetpoint = profile.calculate(0, motorSetpoint, goal);

    motor
        .getClosedLoopController()
        .setReference(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(motorSetpoint.velocity));

    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
    position = Distance.ofBaseUnits(motorEncoder.getPosition(), Meters);
    velocity = LinearVelocity.ofBaseUnits(motorEncoder.getVelocity(), MetersPerSecond);
  }

  @Override
  public void simulationPeriodic() {
    position = setpoint;
  }

  private void setPosition(Distance position) {
    setpoint = position;
  }

  public Command setPositionCommand(Distance position) {
    return Commands.runOnce(() -> this.setPosition(position), this);
  }
}
