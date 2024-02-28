// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class Arm extends Command {
  /** Creates a new Arm. */
  ArmSubsystem m_arm;
  double m_angle;
  // commandbased yazıyorsak eğer subsystemi bu komutta çalıştıracağımızı belirtmemeiz gerekiyor
  public Arm(ArmSubsystem arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = arm;
    this.m_angle = angle;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setIntakeAngle(m_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.AngleMotorLeader.stopMotor();
    m_arm.AngleMotorFollower.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
