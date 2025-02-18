package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.LaserCan;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

@Logged
public class EndEffector extends SubsystemBase {

  private final SparkMax motor;
  private final SparkMaxConfig config;
  private final RelativeEncoder encoder;

  private final SparkClosedLoopController pid;

  private final LaserCan laserCAN;

  double setpoint;
  double velocity;
  Current current;
  Voltage voltage;

  Distance sensorDistance;

  public EndEffector() {
    motor = new SparkMax(EndEffectorConstants.MOTOR_ID, MotorType.kBrushless);

    laserCAN = new LaserCan(EndEffectorConstants.LASERCAN_ID);

    config = new SparkMaxConfig();

    config.inverted(true);

    config.smartCurrentLimit((int) EndEffectorConstants.CURRENT_LIMIT.in(Units.Amps));

    config.encoder.uvwAverageDepth(3);
    config.encoder.uvwMeasurementPeriod(10);

    config.voltageCompensation(12);

    config.idleMode(IdleMode.kBrake);

    config.closedLoop.p(EndEffectorConstants.PID_GAINS.kp);
    config.closedLoop.velocityFF(EndEffectorConstants.FEEDFORWARD_GAINS.kv);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    pid = motor.getClosedLoopController();

    setpoint = 0;

    sensorDistance = Millimeters.of(0);
  }

  @Override
  public void periodic() {
    velocity = encoder.getVelocity();
    current = Current.ofBaseUnits(motor.getOutputCurrent(), Units.Amps);
    voltage = Voltage.ofBaseUnits(motor.getAppliedOutput() * motor.getBusVoltage(), Units.Volts);

    if (laserCAN.getMeasurement() != null) {
      sensorDistance =
          Distance.ofBaseUnits(laserCAN.getMeasurement().distance_mm, Units.Millimeters);
    }
  }

  @Override
  public void simulationPeriodic() {
    velocity = setpoint;
  }

  private void setVelocity(double velocity) {
    pid.setReference(velocity, ControlType.kVelocity);
    setpoint = velocity;
  }

  public Command intakeCommand() {
    return Commands.sequence(
        Commands.race(
            Commands.run(() -> this.setVelocity(EndEffectorConstants.INTAKE_SPEED), this),
            Commands.waitUntil(
                () ->
                    sensorDistance.in(Units.Millimeters)
                        <= EndEffectorConstants.TRIGGER_DISTANCE.in(Units.Millimeters))),
        Commands.runOnce(() -> this.setVelocity(0), this));
  }

  public Command outtakeCommand() {
    return Commands.sequence(
        Commands.race(
            Commands.run(() -> this.setVelocity(EndEffectorConstants.OUTTAKE_SPEED), this),
            Commands.waitUntil(
                () ->
                    !(sensorDistance.in(Units.Millimeters)
                        <= EndEffectorConstants.TRIGGER_DISTANCE.in(Units.Millimeters)))),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> this.setVelocity(0), this));
  }

  public Command setVelocityCommand(double velocity) {
    return Commands.runOnce(() -> this.setVelocity(velocity), this);
  }
}
