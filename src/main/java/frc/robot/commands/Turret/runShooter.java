// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Turret;

public class runShooter extends CommandBase {
  /** Creates a new runShooter. */
  private final Turret turret;
  boolean isFinished = false;

  double minSpeed, currentSpeed, maxSpeed; //debug numbs
  
  public runShooter(Turret subystem) {
    addRequirements(subystem);
    turret = subystem;
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("RPM G", turret.shooter.getRate() * 60);
    //SmartDashboard.putNumber("RPM", turret.shooter.getRate() * 60);
    turret.LeftPower.set(RobotContainer.leftJoystick.getRawAxis(1));
    turret.RightPower.set(-RobotContainer.leftJoystick.getRawAxis(1));
    SmartDashboard.putNumber("Joysticks", RobotContainer.leftJoystick.getRawAxis(1));
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  } 
}
