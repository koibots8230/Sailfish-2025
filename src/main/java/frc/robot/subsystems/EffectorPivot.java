package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EffectorPivotConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class EffectorPivot extends SubsystemBase {

  private final SparkMax motor;
  private final SparkMaxConfig config;

  private final SparkClosedLoopController pid;

  private final RelativeEncoder encoder;

  private final ArmFeedforward feedforward;

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;

  private Angle setpoint;
  private AngularVelocity velocity;
  private Voltage voltage;
  private Current current;
  private Angle position;

  public EffectorPivot() {
    motor = new SparkMax(EffectorPivotConstants.MOTOR_ID, MotorType.kBrushless);


    config = new SparkMaxConfig();
    config.closedLoop.p(EffectorPivotConstants.PID_GAINS.kp);

    config.encoder.positionConversionFactor(EffectorPivotConstants.CONVERSION_FACTOR);
    config.encoder.velocityConversionFactor(EffectorPivotConstants.CONVERSION_FACTOR / 60.0);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
    
    pid = motor.getClosedLoopController();

    profile =
        new TrapezoidProfile(
            new Constraints(
                EffectorPivotConstants.MAX_VELOCITY.in(RotationsPerSecond),
                EffectorPivotConstants.MAX_ACCELERATION.in(RotationsPerSecondPerSecond)));
  
    feedforward =
        new ArmFeedforward(
            EffectorPivotConstants.FEEDFORWARD_GAINS.ks, EffectorPivotConstants.FEEDFORWARD_GAINS.kg, EffectorPivotConstants.FEEDFORWARD_GAINS.kv);

    goal = new TrapezoidProfile.State();
    setpoint = EffectorPivotConstants.INTAKE_POSTIION;
    motorSetpoint = new TrapezoidProfile.State();
  }

  @Override
  public void periodic() {
    goal = new TrapezoidProfile.State(setpoint.in(Radians), 0);

    motorSetpoint =
        profile.calculate(RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds), motorSetpoint, goal);

    pid.setReference(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(motorSetpoint.position, motorSetpoint.velocity));

    position = Angle.ofBaseUnits(encoder.getPosition(), Degrees);
    velocity = AngularVelocity.ofBaseUnits(encoder.getVelocity(), DegreesPerSecond);
    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
  }

  @Override
  public void simulationPeriodic() {
    position = setpoint;
  }

  private void setPosition(Angle position) {
    goal = new TrapezoidProfile.State(position.in(Radians), 0);
    setpoint = position;
  }

  public Command setPositionCommand(Angle position) {
    return Commands.runOnce(() -> this.setPosition(position), this);
  }
}
