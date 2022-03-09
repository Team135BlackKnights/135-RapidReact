// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Turret;

public class runShooter extends CommandBase {
  /** Creates a new runShooter. */
  private final Turret turret;
  boolean isFinished = false;

  double desired, lastOutput, kI, kP, porOut, error, iOut, iTop, iBottom; //pid Numbers
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
    SmartDashboard.putNumber("RPM G", -turret.shooter.getRate() * 60);
    SmartDashboard.putNumber("RPM", -turret.shooter.getRate() * 60);
    SmartDashboard.putNumber("Shooter Error", error);
    desired = limit(-(RobotContainer.manipJoystick.getRawAxis(3) + 1) / 2, .8, 0); 
    SmartDashboard.putNumber("Shooter Desired", desired * 10000);

    error = desired + (desired * .05) + (turret.shooter.getRate() * 60) / 10000;

    iTop = desired * 1.34;
    iBottom = (desired * 1.34) - desired;
    SmartDashboard.putNumber("ITop", iTop);
    SmartDashboard.putNumber("IBottom", iBottom);

    kP = 13; //change when testing
    kI = 5.5; //change when testing

    porOut = error * kP;
    iOut = error * kI;

    //turret.LeftPower.set(outputs());
    turret.RightPower.set(-outputs());
    SmartDashboard.putNumber("Shooter OutPut", outputs());
}

public static double limit(double x, double upperLimit, double lowerLimit) {
  return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
      x;
}

double outputs() {
  if (porOut > iBottom && porOut < iTop) {
      SmartDashboard.putBoolean("I?", true);
      return limit(porOut + iOut, .85, 0);
  } else {
    SmartDashboard.putBoolean("I?", false);
      return limit(porOut, .85, 0);
  }
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