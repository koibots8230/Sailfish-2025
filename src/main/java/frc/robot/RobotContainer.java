// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Swerve;


@Logged
public class RobotContainer {

  private Swerve swerve;
  private final XboxController controller;

  public RobotContainer() {
    swerve = new Swerve();
    controller = new XboxController(0);


    configureBindings();
    defualtCommands();
  }

  private void configureBindings() {


  }

  private void defualtCommands(){
    swerve.setDefaultCommand(swerve.driveFieldRelativeCommand(controller::getLeftY, controller::getLeftX, controller::getRightX));
  }

  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
