package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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

    AngularVelocity setpoint;
    AngularVelocity velocity;
    Current current;
    Voltage voltage;

    public EndEffector() {
        motor = new SparkMax(EndEffectorConstants.MOTOR_ID, MotorType.kBrushless);

        laserCAN = new LaserCan(EndEffectorConstants.LASERCAN_ID);

        config = new SparkMaxConfig();

        config.smartCurrentLimit((int) EndEffectorConstants.CURRENT_LIMIT.in(Units.Amps));

        config.encoder.positionConversionFactor(EndEffectorConstants.CONVERSION_FACTOR);

        config.encoder.uvwAverageDepth(2);
        config.encoder.uvwMeasurementPeriod(20);

        config.voltageCompensation(12);

        config.idleMode(IdleMode.kBrake);

        config.closedLoop.p(EndEffectorConstants.PID_GAINS.kp);
        config.closedLoop.velocityFF(EndEffectorConstants.FEEDFORWARD_GAINS.kv);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();

        pid = motor.getClosedLoopController();

        setpoint = AngularVelocity.ofBaseUnits(0, Units.RPM);
    }

    @Override
    public void periodic() {
        velocity = AngularVelocity.ofBaseUnits(encoder.getVelocity(), Units.RPM);
        current = Current.ofBaseUnits(motor.getOutputCurrent(), Units.Amps);
        voltage = Voltage.ofBaseUnits(motor.getAppliedOutput() * motor.getBusVoltage(), Units.Volts);
    }

    @Override
    public void simulationPeriodic() {
        velocity = setpoint;
    }

    private void setVelocity(AngularVelocity velocity) {
        pid.setReference(velocity.in(Units.RPM), ControlType.kVelocity);

        setpoint = velocity;
    }

    public Command intakeCommand() {
        return Commands.sequence(
            Commands.race(
                Commands.runOnce(() -> this.setVelocity(EndEffectorConstants.INTAKE_SPEED), this),
                Commands.waitUntil(() -> laserCAN.getMeasurement().distance_mm <= EndEffectorConstants.TRIGGER_DISTANCE.in(Units.Millimeter))
            ),
            Commands.runOnce(() -> this.setVelocity(AngularVelocity.ofBaseUnits(0, Units.RPM)), this)
        );
    }
    
    public Command outtakeCommand() {
        return Commands.sequence(
            Commands.race(
                Commands.runOnce(() -> this.setVelocity(EndEffectorConstants.OUTTAKE_SPEED), this),
                Commands.waitUntil(() -> !(laserCAN.getMeasurement().distance_mm <= EndEffectorConstants.TRIGGER_DISTANCE.in(Units.Millimeter)))
            ),
            Commands.runOnce(() -> this.setVelocity(AngularVelocity.ofBaseUnits(0, Units.RPM)), this)
        );
    }

    public Command setVelocityCommand(AngularVelocity velocity) {
        return Commands.runOnce(() -> this.setVelocity(velocity), this);
    }
}