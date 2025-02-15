package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.EndEffector;

public class IntakeCommand {

    public static Command intakeCommand(Intake intake, IntakePivot intakePivot) {
    return Commands.parallel(
    intake.IntakeCommand(IntakeConstants.INTAKE_VELOCITY),
    intakePivot.moveIntakePivotCommand(IntakePivotConstants.OUT_POSITION)
    );
    }

    public static Command intakeStop(Intake intake, IntakePivot intakePivot) {
    return Commands.parallel(
    intake.IntakeCommand(AngularVelocity.ofBaseUnits(0,RPM)),
    intakePivot.moveIntakePivotCommand(IntakePivotConstants.START_POSITION)
    );
    }

    public static Command reveseCommand(Intake intake, IntakePivot intakePivot) {
    return Commands.parallel(
    intake.IntakeCommand(IntakeConstants.REVERSE_INTAKE_VELOCITY),
    intakePivot.moveIntakePivotCommand(IntakePivotConstants.OUT_POSITION)
    );
    }
}