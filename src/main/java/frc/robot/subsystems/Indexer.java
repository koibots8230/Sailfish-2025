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
  private AngularVelocity velocity;
  private AngularVelocity setpoint;

  public Indexer() {
    motor = new SparkMax(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    config = new SparkMaxConfig();
    config.closedLoop.p(IndexerConstants.PID.kp);
    config.closedLoop.velocityFF(IndexerConstants.FEEDFORWARD.kv);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    closedLoopController = motor.getClosedLoopController();

    setpoint = AngularVelocity.ofBaseUnits(0, RPM);
  }

  @Override
  public void periodic() {
    voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
    velocity = AngularVelocity.ofBaseUnits(encoder.getVelocity(), RPM);
  }

  public void simulationPeriodic() {
    velocity = setpoint;
  }

  private void spinIndexer(AngularVelocity setVelocity) {
    closedLoopController.setReference(setVelocity.in(RPM), ControlType.kVelocity);
    setpoint = setVelocity;
  }

  public Command setVelocityCommand(AngularVelocity setVelocity) {
    return Commands.runOnce(() -> this.spinIndexer(setVelocity));
  }
}
