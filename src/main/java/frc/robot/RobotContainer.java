/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.GripperSubsystem;

public class RobotContainer {
  //controllers
  public final CommandJoystick joystick = new CommandJoystick(0);
  public final CommandJoystick joystick2 = new CommandJoystick(1);

  //importing other classes

  private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
  private final GripperSubsystem gripperSubsys = new GripperSubsystem(elevatorSubsys);




  // Autonomous
  SendableChooser<Command> sendableChooser = new SendableChooser<>();

  public RobotContainer() {
    configDefaultCommands();
    configButtonBindings();
    configAutonomous();
  }

  private void configDefaultCommands() {
  }

  private void configButtonBindings() {
    joystick.button(1).whileTrue(gripperSubsys.setAngle(90));
    joystick.button(2).whileTrue(gripperSubsys.testGripperWheel());    
    joystick.button(3).whileTrue(elevatorSubsys.testElevator());
    joystick2.button(1).whileTrue(gripperSubsys.setAngle(270));
    joystick2.button(2).whileTrue(elevatorSubsys.testElevatorNegative());
    joystick2.button(3).whileTrue(gripperSubsys.testGripperWheelNegative());

  }

  private void configAutonomous() {
    SmartDashboard.putData(sendableChooser);
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}