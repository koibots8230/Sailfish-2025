package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Elevator extends SubsystemBase {

  private final SparkMax mainMotor;
  private final SparkMax secondaryMotor;
  private final SparkMaxConfig mainMotorConfig;
  private final SparkMaxConfig secondaryMotorConfig;
  private final TrapezoidProfile profile;
  private final RelativeEncoder motorEncoder;
  private final ElevatorFeedforward feedforward;
  private final DigitalInput HallEffectsSensor;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;
  private Distance setpoint;
  private Distance position;
  private LinearVelocity velocity;
  private Voltage mainVoltage;
  private Voltage secondaryVoltage;
  private Current mainCurrent;
  private Current secondaryCurrent;

  public Elevator() {
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY.in(MetersPerSecond),
                ElevatorConstants.MAX_ACCELRATION.in(MetersPerSecondPerSecond)));

    mainMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    secondaryMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    mainMotorConfig = new SparkMaxConfig();
    mainMotorConfig.idleMode(IdleMode.kBrake);
    mainMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(ElevatorConstants.PID.kp, ElevatorConstants.PID.ki, ElevatorConstants.PID.kd);
    mainMotorConfig.smartCurrentLimit((int) ElevatorConstants.CURRENT_LIMIT.in(Amps));
    mainMotorConfig.alternateEncoder.positionConversionFactor(ElevatorConstants.CONVERSION_FACTOR.in(Meters));
    mainMotorConfig.alternateEncoder.velocityConversionFactor(ElevatorConstants.CONVERSION_FACTOR.in(Meters));
    secondaryMotorConfig = new SparkMaxConfig();
    secondaryMotorConfig.idleMode(IdleMode.kBrake);
    secondaryMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(ElevatorConstants.PID.kp, ElevatorConstants.PID.ki, ElevatorConstants.PID.kd);
    secondaryMotorConfig.smartCurrentLimit((int) ElevatorConstants.CURRENT_LIMIT.in(Amps));
    mainMotor.configure(mainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    secondaryMotor.configure(secondaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorEncoder = mainMotor.getAlternateEncoder();

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.FEEDFORWARD.ks,
            ElevatorConstants.FEEDFORWARD.kg,
            ElevatorConstants.FEEDFORWARD.kv,
            ElevatorConstants.FEEDFORWARD.ka);

    HallEffectsSensor = new DigitalInput(0);

    goal = new TrapezoidProfile.State();
    setpoint = ElevatorConstants.START_SETPOINT;
    motorSetpoint = new TrapezoidProfile.State();

    position = Distance.ofBaseUnits(0, Meters);
    motorEncoder.setPosition(0);
    velocity = LinearVelocity.ofBaseUnits(motorEncoder.getVelocity(), MetersPerSecond);
    mainVoltage = Voltage.ofBaseUnits(mainMotor.getBusVoltage() * mainMotor.getAppliedOutput(), Volts);
    secondaryVoltage = Voltage.ofBaseUnits(secondaryMotor.getBusVoltage() * secondaryMotor.getAppliedOutput(), Volts);
    mainCurrent = Current.ofBaseUnits(mainMotor.getOutputCurrent(), Amps);
    secondaryCurrent = Current.ofBaseUnits(secondaryMotor.getOutputCurrent(), Amps);
  }

  @Override
  public void periodic() {
    goal =
        new TrapezoidProfile.State(
            setpoint.in(Meters), 0);

    motorSetpoint = profile.calculate(RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds), motorSetpoint, goal);

    mainMotor
        .getClosedLoopController()
        .setReference(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(motorSetpoint.velocity));
    
    secondaryMotor
        .getClosedLoopController()
        .setReference(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(motorSetpoint.velocity));

    position = Distance.ofBaseUnits(motorEncoder.getPosition(), Meters);
    velocity = LinearVelocity.ofBaseUnits(motorEncoder.getVelocity(), MetersPerSecond);
    mainVoltage = Voltage.ofBaseUnits(mainMotor.getBusVoltage() * mainMotor.getAppliedOutput(), Volts);
    secondaryVoltage = Voltage.ofBaseUnits(secondaryMotor.getBusVoltage() * secondaryMotor.getAppliedOutput(), Volts);
    mainCurrent = Current.ofBaseUnits(mainMotor.getOutputCurrent(), Amps);
    secondaryCurrent = Current.ofBaseUnits(secondaryMotor.getOutputCurrent(), Amps);
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

  public Command setZeroPositionCommand() {
    return Commands.sequence(
      Commands.race(
        Commands.run(() -> setPosition(Distance.ofBaseUnits(setpoint.in(Millimeters) - 5, Millimeters)), this),
        Commands.waitUntil(() -> HallEffectsSensor.get() == true)
      ),
      Commands.runOnce(() -> motorEncoder.setPosition(0), this)
    );
  }
}
