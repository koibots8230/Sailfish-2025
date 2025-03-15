package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.Hashtable;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
public class TestCommands {
    public static Command testSequence(Swerve swerve, Intake intake, IntakePivot intakePivot, Indexer indexer, Elevator elevator, EndEffector endEffector) {
        return Commands.either(
        Commands.sequence(
            Commands.either(
                Commands.sequence(
                    intake.setVeclocityCommand(IntakeConstants.INTAKE_VELOCITY),
                    Commands.waitSeconds(3),
                    intake.setVeclocityCommand(0)
                ),
                Commands.none(),
                () -> SmartDashboard.getBoolean("TestSequence/Intake", true)
            ),
            Commands.either(
                Commands.sequence(
                    intakePivot.setPositionCommand(IntakePivotConstants.OUT_POSITION),
                    Commands.waitUntil(() -> intakePivot.atSetpoint(IntakePivotConstants.OUT_POSITION)),
                    intakePivot.setPositionCommand(IntakePivotConstants.IN_POSITION),
                    Commands.waitUntil(() -> intakePivot.atSetpoint(IntakePivotConstants.IN_POSITION))
                ),
                Commands.none(),
                () -> SmartDashboard.getBoolean("TestSequence/Intake Pivot", true)
            ),
            Commands.either(
                Commands.sequence(
                    indexer.setVelocityCommand(IndexerConstants.TOP_INDEX_VELOCITY, 0),
                    Commands.waitSeconds(3),
                    indexer.setVelocityCommand(0, 0)
                ),
                Commands.none(),
                () -> SmartDashboard.getBoolean("TestSequence/Indexer Top", true)
            ),
            Commands.either(
                Commands.sequence(
                    indexer.setVelocityCommand(0, IndexerConstants.BOTTOM_INDEX_VELOCITY),
                    Commands.waitSeconds(3),
                    indexer.setVelocityCommand(0, 0)
                ),
                Commands.none(),
                () -> SmartDashboard.getBoolean("TestSequence/Indexer Bottom", true)
            ),
            Commands.either(
                Commands.sequence(
                    elevator.setPositionCommand(ElevatorConstants.L3_POSITION),
                    Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.L3_POSITION)),
                    elevator.setPositionCommand(ElevatorConstants.INTAKE_POSITION),
                    Commands.waitUntil(() -> elevator.atPosition(ElevatorConstants.INTAKE_POSITION))
                ),
                Commands.none(),
                () -> SmartDashboard.getBoolean("TestSequence/Elevator", true)
            ),
            Commands.either(
                Commands.sequence(
                    endEffector.setVelocityCommand(EndEffectorConstants.INTAKE_SPEED),
                    Commands.waitSeconds(3),
                    endEffector.setVelocityCommand(0)
                ),
                Commands.none(),
                () -> SmartDashboard.getBoolean("TestSequence/End Effector", true)
            )
        ), Commands.none(), DriverStation::isTestEnabled);
    }
}
