package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
    
    private CANSparkMax IntakeMotor;

    private Intake() {
        IntakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    }

    //insert intake command here
    
}
