package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.RobotConstants;

@Logged 
public class IntakePivot extends SubsystemBase {
    
    private final SparkMax Motor;
    private final TrapezoidProfile profile;
    private final ArmFeedforward feedforward;
    private final AbsoluteEncoder Encoder;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State motorSetpoint;
    private SparkMaxConfig config;
    
    private Angle setpoint;
    private AngularVelocity velocity;
    private Voltage voltage;
    private Current current;
    private Angle position;
    
    public IntakePivot() {
        Motor = new SparkMax(IntakePivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        Encoder = Motor.getAbsoluteEncoder();

        config = new SparkMaxConfig();
        config.closedLoop.p(IntakePivotConstants.PID_kP);
        config.closedLoop.velocityFF(IntakePivotConstants.PID_kV);
    
        profile = new TrapezoidProfile(new Constraints(
            IntakePivotConstants.MAX_VELOCITY.in(RotationsPerSecond), 
            IntakePivotConstants.MAX_ACCELRATION.in(RotationsPerSecondPerSecond)));
        
        feedforward = new ArmFeedforward(
            IntakePivotConstants.FEEDFORWARD.ks, 
            IntakePivotConstants.FEEDFORWARD.kg, 
            IntakePivotConstants.FEEDFORWARD.kv);

        goal = new TrapezoidProfile.State();
        setpoint =  IntakePivotConstants.START_POSITION;
        motorSetpoint = new TrapezoidProfile.State();

    }

    @Override
    public void periodic() {
        goal =
        new TrapezoidProfile.State(
            setpoint.in(Radians), 0);

    motorSetpoint = profile.calculate(RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds), motorSetpoint, goal);

    Motor
        .getClosedLoopController()
        .setReference(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(motorSetpoint.position, motorSetpoint.velocity));
            
        position = Angle.ofBaseUnits(Encoder.getPosition(), Degrees);
        velocity = AngularVelocity.ofBaseUnits(Encoder.getVelocity(), DegreesPerSecond);
        voltage = Voltage.ofBaseUnits(Motor.getBusVoltage() * Motor.getAppliedOutput(), Volts);
        current = Current.ofBaseUnits(Motor.getOutputCurrent(), Amps);
    }

    @Override
    public void simulationPeriodic() {
        position = setpoint;   
    }

    private void movePivot(Angle position) {
        goal = new TrapezoidProfile.State(position.in(Radians), 0);
        setpoint = position;
    }

    public Command moveIntakePivotCommand(Angle position) {
        return Commands.runOnce(() -> this.movePivot(position), this);
    }

}
