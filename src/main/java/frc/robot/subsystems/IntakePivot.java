import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.RobotConstants;

public class IntakePivot extends SubsystemBase{
    
    private SparkMax pivotMotor;

    private final TrapezoidProfile profile;
    private final ArmFeedforward feedforward;
    private final AbsoluteEncoder motorEncoder;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State motorSetpoint;

    private Angle setpoint;
    private AngularVelocity velocity;
    private Voltage voltage;
    private Current current;
    private Angle position;
    
    public IntakePivot() {
        SparkMax pivotMotor = new SparkMax(IntakePivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    
        profile = new TrapezoidProfile(new Constraints(0, 0));
        feedforward = new ArmFeedforward(0, 0, 0);

        goal = new TrapezoidProfile.State();
        setpoint = IntakePivotConstants.START_SETPOINT;
        motorSetpoint = new TrapezoidProfile.State();

        position = Angle.ofBaseUnits(motorEncoder.getPosition(), Degrees);
        velocity = AngularVelocity.ofBaseUnits(motorEncoder.getVelocity(), DegreesPerSecond);
        voltage = Voltage.ofBaseUnits(pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput(), Volts);
        current = Current.ofBaseUnits(pivotMotor.getOutputCurrent(), Amps);
    
    }

    @Override
    public void periodic() {
        motorSetpoint = profile.calculate(0, motorSetpoint, goal);

        pivotMotor
            .getClosedLoopController()
            .setReference(
                motorSetpoint.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward.calculate(motorSetpoint.position, motorSetpoint.velocity));
            
        position = Angle.ofBaseUnits(motorEncoder.getPosition(), Degrees);
        velocity = AngularVelocity.ofBaseUnits(motorEncoder.getVelocity(), DegreesPerSecond);
        voltage = Voltage.ofBaseUnits(pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput(), Volts);
        current = Current.ofBaseUnits(pivotMotor.getOutputCurrent(), Amps);
    
    }

    private void movePivot(double position) {
        goal = new TrapezoidProfile.State(setpoint.in(Degrees), 0);
        

    }

    public Command moveIntakePivotCommadn(double position) {
        return Commands.runOnce(() -> this.movePivot(position), this);
    }

}
