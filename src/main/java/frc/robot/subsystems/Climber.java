package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Climber extends SubsystemBase {
  @NotLogged private final SparkMax motor;
  @NotLogged private final SparkMaxConfig config;
  @NotLogged private final AbsoluteEncoder encoder;

  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State motorSetpoint;
  private double setpoint;
  private double velocity;
  private Voltage voltage;
  private Current current;
  private double position;
  double speed;

  public Climber() {
    motor = new SparkMax(ClimberConstants.MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ClimberConstants.PID.kp, ClimberConstants.PID.ki, ClimberConstants.PID.kd);

    config.smartCurrentLimit((int) ClimberConstants.CURRENT_LIMIT.in(Amps));

    config.absoluteEncoder.inverted(true);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    goal = new TrapezoidProfile.State();
    setpoint = ClimberConstants.START_POSITION;
    motorSetpoint = new TrapezoidProfile.State();

    speed = ClimberConstants.HIGH_SPEED;

    position = encoder.getPosition();
    velocity = encoder.getVelocity();

    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
  }

  @Override
  public void periodic() {
    if (position < setpoint) {
      motor.set(ClimberConstants.HIGH_SPEED);    
    } else {
      motor.set(0);
    }

    position = encoder.getPosition();
    velocity = encoder.getVelocity();

    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
  }

  @Override
  public void simulationPeriodic() {
    position = motorSetpoint.position;
  }

  private void setAngle(double angle) {
    setpoint = angle;
  }

  public Command setAngleCommand(double angle) {
    return Commands.runOnce(() -> this.setAngle(angle), this);
  }
}
