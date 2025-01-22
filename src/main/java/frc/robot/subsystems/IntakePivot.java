package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
    
    private Angle setpoint;
    private AngularVelocity velocity;
    private Voltage voltage;
    private Current current;
    private Angle position;
    
    public IntakePivot() {
        Motor = new SparkMax(IntakePivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        Encoder = Motor.getAbsoluteEncoder();
    
        profile = new TrapezoidProfile(new Constraints(0, 0));
        feedforward = new ArmFeedforward(0, 0, 0);

        goal = new TrapezoidProfile.State();
        setpoint =  IntakePivotConstants.START_POSITION;
        motorSetpoint = new TrapezoidProfile.State();
    }

    @Override
    public void periodic() {
        goal =
        new TrapezoidProfile.State(
            setpoint.in(Degrees), RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds));

    motorSetpoint = profile.calculate(0, motorSetpoint, goal);

    Motor
        .getClosedLoopController()
        .setReference(
            motorSetpoint.position,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(motorSetpoint.velocity));
            
        position = Angle.ofBaseUnits(Encoder.getPosition(), Degrees);
        velocity = AngularVelocity.ofBaseUnits(Encoder.getVelocity(), DegreesPerSecond);
        voltage = Voltage.ofBaseUnits(Motor.getBusVoltage() * Motor.getAppliedOutput(), Volts);
        current = Current.ofBaseUnits(Motor.getOutputCurrent(), Amps);
    
    }

    @Override
    public void simulationPeriodic() {
        //????
    }

    private void movePivot(Angle position) {
        goal = new TrapezoidProfile.State(position.in(Degrees), 0);
    }

    public Command moveIntakePivotCommand(Angle position) {
        return Commands.runOnce(() -> this.movePivot(position), this);
    }

}
