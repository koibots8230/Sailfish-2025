package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private SparkMax IntakeMotor;
    private SparkMaxConfig config;
    private SparkClosedLoopController closedLoopController;

    private Intake() {
        SparkMax IntakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        
        config = new SparkMaxConfig();
        config.closedLoop.p(IntakeConstants.PID_kP);
        config.closedLoop.velocityFF(IntakeConstants.PID_kV);

        IntakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        closedLoopController = IntakeMotor.getClosedLoopController();
    }

    private void spinIntake(double velocity) {
        closedLoopController.setReference(velocity, ControlType.kVelocity);
    
    }

    public Command IntakeCommand (double velocity){
        return Commands.runOnce(() -> this.spinIntake(velocity), this);
    }
    
}
