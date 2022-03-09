// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Turret;

public class angleHood extends CommandBase {
  
  Turret turret;
  
  double MaxHight = 3000; //~47 degrees

  boolean isFinished = false;
  double minSpeed, currentSpeed, maxSpeed; //debug numbs
  
  public angleHood(Turret m_Turret) {
    addRequirements(m_Turret);
    turret = m_Turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Hood Running", true);
    if (RobotContainer.manipJoystick.getPOV() == 0){
      turret.hoodMotor.set(.2);
    }
    else if (RobotContainer.manipJoystick.getPOV() == 180){
      turret.hoodMotor.set(-.2);
    }
    else {
      turret.hoodMotor.set(0);
      isFinished = true;
    }
  
  }

  public static double limit(double x, double upperLimit, double lowerLimit) {
    return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
        x;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.hoodMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Hood Running", false);
    return isFinished;
  }
}
