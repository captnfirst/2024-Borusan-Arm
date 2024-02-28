// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {

  //Joystick ve subsystem tanımlamasını burada yapıyoruz

  Joystick stick_0 = new Joystick(0);
  ArmSubsystem arm = new ArmSubsystem();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // butona basıldığı sürece çalışmasını sağlıyoruz
    new JoystickButton(stick_0, 1).whileTrue(new Arm(arm, 10));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
