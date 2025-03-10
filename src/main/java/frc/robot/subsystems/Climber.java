package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Climber extends SubsystemBase{
    @NotLogged private final SparkMax motor;
    @NotLogged private final SparkMaxConfig config;
    @NotLogged private final AbsoluteEncoder encoder;
    @NotLogged private final SparkClosedLoopController pid;
    @NotLogged private final SimpleMotorFeedforward feedforward;
    @NotLogged private final TrapezoidProfile profile; 

    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State motorSetpoint;
    private Angle setpoint;
    private AngularVelocity velocity;
    private Voltage voltage;
    private Current current;
    private Angle position;

    public Climber() {
        profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ClimberConstants.MAX_VELOCITY.in(RotationsPerSecond),
                ClimberConstants.MAX_ACCELERATION.in(RotationsPerSecondPerSecond)));
        
        motor = new SparkMax(0, MotorType.kBrushless);
        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ClimberConstants.PID.kp, ClimberConstants.PID.ki, ClimberConstants.PID.kd);

        config.smartCurrentLimit((int) ClimberConstants.CURRENT_LIMIT.in(Amps));

        config.absoluteEncoder.positionConversionFactor(ClimberConstants.CONVERSION_FACTOR);
        config.absoluteEncoder.velocityConversionFactor(ClimberConstants.RPM_TO_RPS_FACTOR);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getAbsoluteEncoder();

        pid = motor.getClosedLoopController();

        feedforward = new SimpleMotorFeedforward(
            ClimberConstants.FEEDFORWARD.ks, 
            ClimberConstants.FEEDFORWARD.kg,
            ClimberConstants.FEEDFORWARD.kv,
            ClimberConstants.FEEDFORWARD.ka);
        
        goal = new TrapezoidProfile.State();
        setpoint = ClimberConstants.START_POSITION;
        motorSetpoint = new TrapezoidProfile.State();

        position = Radians.of(0);;
        velocity = AngularVelocity.ofBaseUnits(encoder.getVelocity(), RotationsPerSecond);

        voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
        current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
    }

    @Override
    public void periodic() {
        goal = new TrapezoidProfile.State(setpoint.in(Radians), 0);

        motorSetpoint =
            profile.calculate(RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds), motorSetpoint, goal);
        
        pid.setReference(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(motorSetpoint.position, motorSetpoint.velocity));
        
        position = Radians.of(encoder.getPosition());
        velocity = AngularVelocity.ofBaseUnits(encoder.getVelocity(), RotationsPerSecond);

        voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
        current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
    }

    @Override
    public void simulationPeriodic() {
        position = setpoint;
    }

    private void setAngle(Angle angle) {
        setpoint = angle;
    }

    public Command setAngleCommand(Angle angle) {
        return Commands.runOnce(() -> this.setAngle(angle), this);
    }
}
