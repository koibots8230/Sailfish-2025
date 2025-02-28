package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
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
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

@Logged
public class Indexer extends SubsystemBase {

  @NotLogged private final SparkMax motor;

  @NotLogged private final SparkMaxConfig config;

  @NotLogged private final RelativeEncoder encoder;
  
  @NotLogged private final SparkClosedLoopController closedLoopController;

  private double setpoint;
  private double velocity;
  private Voltage voltage;
  private Current current;

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
    velocity = setpoint;
  }

  private void setVelocity(double setVelocity) {
    closedLoopController.setReference(setVelocity, ControlType.kVelocity);
    setpoint = setVelocity;
  }

  public Command setVelocityCommand(double setVelocity) {
    return Commands.runOnce(() -> this.setVelocity(setVelocity));
  }
}
