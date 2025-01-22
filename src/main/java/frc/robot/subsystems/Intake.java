package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged 
public class Intake extends SubsystemBase {
    
    private final RelativeEncoder motorEncoder;
    private final SparkMax intakeMotor;
    private SparkMaxConfig config;
    private SparkClosedLoopController closedLoopController;

    private AngularVelocity setpoint;
    private AngularVelocity velocity;
    private Voltage voltage;
    private Current current;


    public Intake() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        motorEncoder = intakeMotor.getEncoder();
        
        config = new SparkMaxConfig();
        config.closedLoop.p(IntakeConstants.PID_kP);
        config.closedLoop.velocityFF(IntakeConstants.PID_kV);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        closedLoopController = intakeMotor.getClosedLoopController();

        setpoint = AngularVelocity.ofBaseUnits(0, RPM);
    }

    @Override
    public void periodic() {
        velocity = AngularVelocity.ofBaseUnits(motorEncoder.getVelocity(), RPM);
        voltage = Voltage.ofBaseUnits(intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput(), Volts);
        current = Current.ofBaseUnits(intakeMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void simulationPeriodic() {
        velocity = setpoint;
    }

    private void spinIntake(AngularVelocity setVelocity) {
        closedLoopController.setReference(setVelocity.in(RPM), ControlType.kVelocity);
        setpoint = setVelocity;
    }

    public Command IntakeCommand(AngularVelocity setVelocity){
        return Commands.runOnce(() -> this.spinIntake(setVelocity), this);
    }
    
}
