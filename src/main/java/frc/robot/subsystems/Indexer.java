package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class Indexer extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig motorConfig;
    private final SparkClosedLoopController closedLoopController;

    final Voltage voltage;
    final Current current;
    final AngularVelocity velocity;
    final AngularVelocity setpoint;
    
    public Indexer(){
        motor = new SparkMax(IndexerConstants.MOTOR_ID, MotorType.kBrushless);
            
        motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.p(IndexerConstants.PID.kp);
        motorConfig.closedLoop.velocityFF(IndexerConstants.FEEDFORWARD.kv);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.ClosedLoop.p(IndexerConstants.PID.kp);
        config.ClosedLoop.velocityFF(IndexerConstants.FEEDFORWARD.kv);

        encoder = motor.getEncoder(); 
    
        }
    @Override
    public void periodic(){
        voltage = Voltage.ofBaseUnits(motor.getBusVoltage() * motor.getAppliedOutput(), Volts);
        current = Current.ofBaseUnits(motor.getOutputCurrent(), Amps);
        velocity = AngularVelocity.ofBaseUnits(encoder.getVelocity(), RPM);
        }
    
    private void SpinIndexer(double velocity){
        

    }



}
