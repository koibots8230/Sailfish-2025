package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
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

  @NotLogged private final SparkMax mainMotor;

  @NotLogged private final DigitalInput hallEffectSensor;

  @NotLogged private final SparkMaxConfig mainMotorConfig;

  @NotLogged private final RelativeEncoder encoder;

  @NotLogged private final SparkClosedLoopController pid;

  @NotLogged private final ElevatorFeedforward feedforward;

  @NotLogged private final TrapezoidProfile profile;

  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;

  private Distance setpoint;
  private Distance position;
  private LinearVelocity velocity;
  private Voltage mainVoltage;
  private Voltage secondaryVoltage;
  private Current mainCurrent;

  public Elevator() {
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY.in(MetersPerSecond),
                ElevatorConstants.MAX_ACCELRATION.in(MetersPerSecondPerSecond)));

    mainMotor = new SparkMax(ElevatorConstants.MAIN_MOTOR_ID, MotorType.kBrushless);

    mainMotorConfig = new SparkMaxConfig();

    mainMotorConfig.idleMode(IdleMode.kCoast);

    mainMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(ElevatorConstants.PID.kp, ElevatorConstants.PID.ki, ElevatorConstants.PID.kd);

    mainMotorConfig.smartCurrentLimit((int) ElevatorConstants.CURRENT_LIMIT.in(Amps));

    mainMotorConfig.alternateEncoder.positionConversionFactor(ElevatorConstants.CONVERSION_FACTOR);
    mainMotorConfig.alternateEncoder.velocityConversionFactor(
        ElevatorConstants.CONVERSION_FACTOR / 60.0);
    mainMotorConfig.alternateEncoder.inverted(true);

    mainMotor.configure(
        mainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = mainMotor.getAlternateEncoder();

    pid = mainMotor.getClosedLoopController();

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.FEEDFORWARD.ks,
            ElevatorConstants.FEEDFORWARD.kg,
            ElevatorConstants.FEEDFORWARD.kv,
            ElevatorConstants.FEEDFORWARD.ka);

    hallEffectSensor = new DigitalInput(ElevatorConstants.HALL_EFFECTS_SENSOR);

    goal = new TrapezoidProfile.State();
    setpoint = ElevatorConstants.INTAKE_POSITION;
    motorSetpoint = new TrapezoidProfile.State();

    position = Distance.ofBaseUnits(0, Meters);
    velocity = LinearVelocity.ofBaseUnits(encoder.getVelocity(), MetersPerSecond);

    mainVoltage =
        Voltage.ofBaseUnits(mainMotor.getBusVoltage() * mainMotor.getAppliedOutput(), Volts);

    mainCurrent = Current.ofBaseUnits(mainMotor.getOutputCurrent(), Amps);
  }

  @Override
  public void periodic() {
    goal = new TrapezoidProfile.State(setpoint.in(Meters), 0);

    motorSetpoint =
        profile.calculate(RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds), motorSetpoint, goal);

    pid.setReference(
        motorSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(motorSetpoint.velocity));

    position = Distance.ofBaseUnits(encoder.getPosition(), Meters);
    velocity = LinearVelocity.ofBaseUnits(encoder.getVelocity(), MetersPerSecond);

    mainVoltage =
        Voltage.ofBaseUnits(mainMotor.getBusVoltage() * mainMotor.getAppliedOutput(), Volts);
    
    mainCurrent = Current.ofBaseUnits(mainMotor.getOutputCurrent(), Amps);
  }

  public boolean atPosition(Distance desieredPostion) {
    return (position.gte(desieredPostion.minus(Meters.of(.025)))
        && position.lte(desieredPostion.plus(Meters.of(.025))));
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

  public Command setZeroPositionCommand() { // TODO: Untested
    return Commands.sequence(
        Commands.race(
            Commands.run(
                () ->
                    setPosition(Distance.ofBaseUnits(setpoint.in(Millimeters) - 0.05, Millimeters)),
                this),
            Commands.waitUntil(() -> hallEffectSensor.get() == true)),
        Commands.runOnce(() -> encoder.setPosition(0), this));
  }
}
