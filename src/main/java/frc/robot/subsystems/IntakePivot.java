import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivot {
    
    private CANSparkMax pivotMotor;

    public IntakePivot() {
        pivotMotor = new CANSparkMax(IntakePivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    }

    private void movePivot(double position) {
        this.setGoal(new State(position, 0));
    }

    public Command moveIntakePivotCommadn(double position) {
        return Commands.runOnce(() -> this.movePivot(position), this);
    }

}
