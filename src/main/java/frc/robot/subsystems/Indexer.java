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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

@Logged
public class Indexer extends SubsystemBase {

  private final SparkMax topMotor;
  private final SparkMax bottomMotor;

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  private final SparkMaxConfig topConfig;
  private final SparkMaxConfig bottomConfig;

  private final SparkClosedLoopController topClosedLoopController;
  private final SparkClosedLoopController bottomClosedLoopController;

  private Voltage topVoltage;
  private Current topCurrent;
  private double topVelocity;
  private double topSetpoint;

  private Voltage bottomVoltage;
  private Current bottomCurrent;
  private double bottomVelocity;
  private double bottomSetpoint;

  public Indexer() {
    topMotor = new SparkMax(IndexerConstants.TOP_ID, MotorType.kBrushless);
    bottomMotor = new SparkMax(IndexerConstants.BOTTOM_ID, MotorType.kBrushless);

    topConfig = new SparkMaxConfig();
    topConfig.closedLoop.p(IndexerConstants.TOP_PID.kp);
    topConfig.closedLoop.velocityFF(IndexerConstants.TOP_FF.kv);
    topConfig.inverted(true);
    topConfig.smartCurrentLimit(60);

    topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topEncoder = topMotor.getEncoder();

    topClosedLoopController = topMotor.getClosedLoopController();

    bottomConfig = new SparkMaxConfig();
    bottomConfig.closedLoop.p(IndexerConstants.BOTTOM_PID.kp);
    bottomConfig.closedLoop.velocityFF(IndexerConstants.BOTTOM_FF.kv);
    bottomConfig.inverted(true);
    bottomConfig.smartCurrentLimit(60);

    bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    bottomEncoder = bottomMotor.getEncoder();

    bottomClosedLoopController = bottomMotor.getClosedLoopController();

    topSetpoint = 0.0;
    bottomSetpoint = 0.0;
  }

  @Override
  public void periodic() {
    topVoltage = Voltage.ofBaseUnits(topMotor.getBusVoltage() * topMotor.getAppliedOutput(), Volts);
    topCurrent = Current.ofBaseUnits(topMotor.getOutputCurrent(), Amps);
    topVelocity = topEncoder.getVelocity();

    bottomVoltage = Voltage.ofBaseUnits(bottomMotor.getBusVoltage() * bottomMotor.getAppliedOutput(), Volts);
    bottomCurrent = Current.ofBaseUnits(bottomMotor.getOutputCurrent(), Amps);
    bottomVelocity = bottomEncoder.getVelocity();
  }

  public void simulationPeriodic() {
    // velocity = setpoint.mutableCopy();
  }

  private void spinIndexer(double setVelocity) {
    topClosedLoopController.setReference(setVelocity, ControlType.kVelocity);
    bottomClosedLoopController.setReference(setVelocity, ControlType.kVelocity);

    topSetpoint = setVelocity;
    bottomSetpoint = setVelocity;
  }

  public Command setVelocityCommand(double setVelocity) {
    return Commands.runOnce(() -> this.spinIndexer(setVelocity));
  }
}
