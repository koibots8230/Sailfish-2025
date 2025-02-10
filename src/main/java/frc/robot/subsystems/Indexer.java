package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class Indexer extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig motorConfig;


    public Indexer(){
        motor = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);
        
        motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.p(IndexerConstants.PID.kp);
        motorConfig.closedLoop.velocityFF(IndexerConstants.FEEDFORWARD.kv);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder(); 

    }
    
    private void SpinIndexer(double velocity){
    
    }



}
