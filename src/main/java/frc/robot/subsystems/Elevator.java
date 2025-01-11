package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

@Logged
public class Elevator extends SubsystemBase {

  private final SparkMax motor;
  private SparkMaxConfig motorConfig;
  private TrapezoidProfile profile;
  private AbsoluteEncoder motorEncoder;
  private TrapezoidProfile.State goal;
  private double setpoint;
  private TrapezoidProfile.State motorSetpoint;
  private Distance position;
  private LinearVelocity velocity;

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
        .pid(
            ElevatorConstants.PID.getP(),
            ElevatorConstants.PID.getI(),
            ElevatorConstants.PID.getD());
    motorConfig.absoluteEncoder.positionConversionFactor(
        ElevatorConstants.POSITION_CONVERSION_FACTOR);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorEncoder = motor.getAbsoluteEncoder();

    goal = new TrapezoidProfile.State();
    setpoint = ElevatorConstants.START_SETPOINT;
    motorSetpoint = new TrapezoidProfile.State();

    position = Distance.ofBaseUnits(motorEncoder.getPosition(), Meters);
    velocity = LinearVelocity.ofBaseUnits(motorEncoder.getVelocity(), MetersPerSecond);
  }

  @Override
  public void periodic() {
    goal = new TrapezoidProfile.State(setpoint, 0);

    motorSetpoint = profile.calculate(0, motorSetpoint, goal);

    motor.getClosedLoopController().setReference(motorSetpoint.position, ControlType.kPosition);
  }

  private void setPosition(Distance position) {
    setpoint = position.in(Meters);
  }

  public Command setPositionCommand(Distance position) {
    return Commands.runOnce(() -> this.setPosition(position), this);
  }
}
