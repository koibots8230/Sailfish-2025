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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
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
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class IntakePivot extends SubsystemBase {

  private final SparkMax motor;
  private final TrapezoidProfile profile;
  private final SimpleMotorFeedforward feedforward;
  private final RelativeEncoder encoder;
  private DigitalInput limitSwitch;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;
  private SparkMaxConfig config;

  private Angle setpoint;
  private AngularVelocity velocity;
  private Voltage voltage;
  private Current current;
  private Angle position;

  public IntakePivot() {
    motor = new SparkMax(IntakePivotConstants.INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    limitSwitch = new DigitalInput(IntakePivotConstants.INTAKE_PIVOT_SWITCH_CHANEL);

    config = new SparkMaxConfig();
    config.closedLoop.p(IntakePivotConstants.PID.kp);
    config.closedLoop.velocityFF(IntakePivotConstants.FEEDFORWARD.kv);

    config.encoder.positionConversionFactor(IntakePivotConstants.POSITION_CONVERSION_FACTOR);
    config.encoder.velocityConversionFactor(IntakePivotConstants.VELCOITY_CONVERSION_FACTOR);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

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

    motor
        .getClosedLoopController()
        .setReference(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(motorSetpoint.velocity));

    position = Angle.ofBaseUnits(encoder.getPosition(), Degrees);
    velocity = AngularVelocity.ofBaseUnits(encoder.getVelocity(), DegreesPerSecond);
    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);

    if (limitSwitch.get()) {
      position = IntakePivotConstants.IN_POSITION;
      encoder.setPosition(position.in(Radians));
    }
  }

  public boolean atPosition() {
    return (position.gte(IntakePivotConstants.IN_POSITION.minus(Angle.ofBaseUnits(0.025, Radians)))
        && position.lt(
            IntakePivotConstants.IN_POSITION.plus(
                Angle.ofBaseUnits(
                    0.025,
                    Radians)))); // TODO CHANGE THE PLUS AND MINUS VALUES HERE TO SOMTHING RELEVENT
  }

  @Override
  public void simulationPeriodic() {
    position = setpoint;
  }

  private void movePivot(Angle position) {
    goal = new TrapezoidProfile.State(position.in(Radians), 0);
    setpoint = position;
  }

  public Command moveIntakePivotCommand(Angle position) {
    return Commands.runOnce(() -> this.movePivot(position), this);
  }
}
