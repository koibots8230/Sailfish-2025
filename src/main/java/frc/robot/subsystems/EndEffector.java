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
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.EndEffectorState;
import frc.robot.Constants.EndEffectorConstants;

@Logged
public class EndEffector extends SubsystemBase {

  @NotLogged private final SparkMax motor;

  @NotLogged private final LaserCan laserCAN;

  @NotLogged private final SparkMaxConfig config;

  @NotLogged private final RelativeEncoder encoder;

  @NotLogged private final SparkClosedLoopController pid;

  double setpoint;
  double velocity;
  Current current;
  Voltage voltage;

  Distance sensorDistance;

  EndEffectorState state;

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

    config.closedLoop.p(EndEffectorConstants.PID.kp);
    config.closedLoop.velocityFF(EndEffectorConstants.FEEDFORWARD.kv);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    pid = motor.getClosedLoopController();

    setpoint = 0;

    sensorDistance =
        (laserCAN.getMeasurement() == null)
            ? Millimeters.of(0)
            : Distance.ofBaseUnits(laserCAN.getMeasurement().distance_mm, Units.Millimeters);
    ;

    state = this.hasCoral() ? EndEffectorState.hasCoral : EndEffectorState.noCoral;
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

    if (hasCoral() && !RobotState.isDisabled()) {
      state = EndEffectorState.hasCoral;
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

  private void holdCoral() {
    if (state == EndEffectorState.hasCoral
        && !(sensorDistance.in(Units.Millimeters)
            <= EndEffectorConstants.TRIGGER_DISTANCE.in(Units.Millimeters))) {
      setpoint = EndEffectorConstants.HOLDING_SPEED;
      pid.setReference(EndEffectorConstants.HOLDING_SPEED, ControlType.kVelocity);
    } else if (sensorDistance.in(Units.Millimeters)
            >= EndEffectorConstants.TRIGGER_DISTANCE.in(Units.Millimeters)
        && sensorDistance.in(Units.Millimeters)
            <= EndEffectorConstants.ALGAE_REMOVER_DISTANCE.in(Units.Millimeters)) {
      this.releaseAlgaeRemover().schedule();
    } else {
      setpoint = 0;
      pid.setReference(0, ControlType.kVelocity);
    }
  }

  private void setState(EndEffectorState state) {
    this.state = state;
  }

  public boolean hasCoral() {
    return sensorDistance.in(Units.Millimeters)
        <= EndEffectorConstants.TRIGGER_DISTANCE.in(Units.Millimeters);
  }

  public Command intakeCommand() {
    return Commands.sequence(
        Commands.race(
            Commands.run(() -> this.setVelocity(EndEffectorConstants.INTAKE_SPEED), this),
            Commands.waitUntil(
                () ->
                    sensorDistance.in(Units.Millimeters)
                        <= EndEffectorConstants.TRIGGER_DISTANCE.in(Units.Millimeters))),
        Commands.runOnce(() -> this.setVelocity(0), this),
        Commands.runOnce(() -> this.setState(EndEffectorState.hasCoral), this));
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
        Commands.runOnce(() -> this.setVelocity(0), this),
        Commands.runOnce(() -> this.setState(EndEffectorState.noCoral), this));
  }

  public Command releaseAlgaeRemover() {
    return Commands.sequence(
        this.setVelocityCommand(-EndEffectorConstants.HOLDING_SPEED),
        Commands.waitSeconds(0.3),
        this.setVelocityCommand(EndEffectorConstants.HOLDING_SPEED),
        Commands.waitUntil(this::hasCoral),
        this.setVelocityCommand(0));
  }

  public Command holdCoralCommand() {
    return Commands.run(this::holdCoral, this);
  }

  public Command removeAlgaeCommand() {
    return Commands.run(() -> this.setVelocity(EndEffectorConstants.ALGAE_REMOVAL_SPEED), this);
  }

  public Command setVelocityCommand(double velocity) {
    return Commands.runOnce(() -> this.setVelocity(velocity), this);
  }

  public Command setStateCommand(EndEffectorState state) {
    return Commands.runOnce(() -> this.setState(state), this);
  }
}
