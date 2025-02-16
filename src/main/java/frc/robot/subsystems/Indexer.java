package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

@Logged
public class Indexer extends SubsystemBase {

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController closedLoopController;

  private Voltage voltage;
  private Current current;
  private double velocity;
  private double setpoint;

  public Indexer() {
    motor = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();
    config.closedLoop.p(IndexerConstants.PID.kp);
    config.closedLoop.velocityFF(IndexerConstants.FEEDFORWARD.kv);
    config.inverted(true);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    closedLoopController = motor.getClosedLoopController();

    setpoint = 0.0;

    velocity = 0.0;
  }

  @Override
  public void periodic() {
    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
    velocity = encoder.getVelocity();
  }

  public void simulationPeriodic() {
    // velocity = setpoint.mutableCopy();
  }

  private void spinIndexer(double setVelocity) {
    closedLoopController.setReference(setVelocity, ControlType.kVelocity);
    setpoint = setVelocity;
  }

  public Command indexerCommand(double setVelocity) {
    return Commands.runOnce(() -> this.spinIndexer(setVelocity));
  }
}
