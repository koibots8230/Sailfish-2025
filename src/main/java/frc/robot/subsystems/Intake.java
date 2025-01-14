package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake {
    
    private SparkMax IntakeMotor;

    private Intake() {
        SparkMax IntakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    }

    public void intake() {
        //make it spin here
    
    }

    //add command factory here
    
}
