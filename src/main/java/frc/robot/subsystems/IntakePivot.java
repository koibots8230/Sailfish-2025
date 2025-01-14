import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.RobotConstants;

public class IntakePivot extends SubsystemBase{
    
    private SparkMax pivotMotor;

    private final TrapezoidProfile profile;
    private final ElevatorFeedforward feedforward;
    private TrapezoidProfile.State goal;
    private Angle setpoint;
    private TrapezoidProfile.State motorSetpoint;
    

    public IntakePivot() {
        SparkMax pivotMotor = new SparkMax(IntakePivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    
        goal = new TrapezoidProfile.State();
        setpoint = IntakePivotConstants.START_SETPOINT;
        motorSetpoint = new TrapezoidProfile.State();
    
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
                feedforward.calculate(motorSetpoint.velocity));
    
    }

    private void movePivot(double position) {
        goal = new TrapezoidProfile.State(setpoint.in(Degrees), 0);

    }

    public Command moveIntakePivotCommadn(double position) {
        return Commands.runOnce(() -> this.movePivot(position), this);
    }

}
