package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

@Logged
public class Intake extends SubsystemBase {

  private final RelativeEncoder encoder;
  private final SparkFlex motor;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController closedLoopController;

  private double setpoint;
  private double velocity;
  private Voltage voltage;
  private Current current;

  public Intake() {
    motor = new SparkFlex(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();
    config.closedLoop.p(IntakeConstants.PID.kp);
    config.closedLoop.velocityFF(IntakeConstants.FEEDFORWARD.kv);

    config.inverted(true);

    config.smartCurrentLimit((int) IntakeConstants.CURRENT_LIMIT.in(Amps));

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    closedLoopController = motor.getClosedLoopController();

    setpoint = 0;
  }

  @Override
  public void periodic() {
    velocity = encoder.getVelocity();
    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
  }

  @Override
  public void simulationPeriodic() {
    velocity = setpoint;
  }

  private void spinIntake(double setVelocity) {
    closedLoopController.setReference(setVelocity, ControlType.kVelocity);
    setpoint = setVelocity;
  }

  public Command setVeclocityCommand(double setVelocity) {
    return Commands.runOnce(() -> this.spinIntake(setVelocity), this);
  }
}
