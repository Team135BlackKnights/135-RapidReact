// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Turret;

public class angleHood extends CommandBase {
  /** Creates a new angleHood. */
  Turret turret;
  
  double MaxHight = 10000;

  boolean isFinished = false;

  double desired, lastOutput, kI, kP, porOut, error, iOut, iTop, iBottom; //pid Numbers
  double minSpeed, currentSpeed, maxSpeed; //debug numbs
  
  public angleHood(Turret m_Turret) {
    addRequirements(m_Turret);
    turret = m_Turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    desired = -(RobotContainer.manipJoystick.getRawAxis(3) - 1); //change this num with testing
    error = desired - (turret.hoodHight.get() / MaxHight * 2);

    iTop = desired * .34;
    iBottom = desired - (desired * 1.34);
    kP = .2; //change when testing
    kI = -.08; //change when testing

    porOut = error * kP;
    iOut = error * kI;

    turret.hoodMotor.set(outputs());

    SmartDashboard.putNumber("Hood Output", outputs());

    if (Math.abs(error) < 20) {
      isFinished = true;
    }
  }

  public static double limit(double x, double upperLimit, double lowerLimit) {
    return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
        x;
  }
  
  double outputs() {
    if (porOut > iBottom && porOut < iTop) {
        return limit(porOut + iOut, .8, 0);
    } else {
        return limit(porOut, .8, 0);
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
