// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret.Turret;

public class runShooter extends CommandBase {
  /** Creates a new runShooter. */
  private final Turret turret;
  boolean isFinished = false;

  double error, desired, Kp, Ki, integralTop, integralBottom; //function Numbs
  double minSpeed, currentSpeed, maxSpeed; //debug numbs
  
  public runShooter(Turret subystem, double m_Desired) {
    addRequirements(subystem);
    turret = subystem;
    desired = m_Desired;
  }

  // Called when the command is initially scheduled
  /*
  @Override
  public void initialize() {

    turret.resetEncoders();
    error = desired - turret.shooter.get();

    integralTop = desired * 1.34;
    integralBottom = desired - (desired * 1.34);

    Kp = .05;
    Ki = .01;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(error) < 3) {isFinished = true;}
    // check to see if the speed is close enuff 

    error = desired - turret.shooter.get();
    //find the new error

    if (error < integralTop && error > integralBottom){
      turret.setPower(limit(error * Ki + error * Kp, .89, -.89));
    }
    else {
     turret.setPower(limit(error * Kp, 89, -.89)); 
    }
    //set the turrets speed to the PID loop

    if (currentSpeed > maxSpeed) {maxSpeed = currentSpeed;}
    if (minSpeed < currentSpeed) {minSpeed = currentSpeed;}

    SmartDashboard.putNumber("Shooter Speed Graph", currentSpeed);
    SmartDashboard.putNumber("Shooter Speed", currentSpeed);
    SmartDashboard.putNumber("Max Shooter Speed", maxSpeed);
    SmartDashboard.putNumber("Min Shooter Speed", minSpeed);
    //debug log the speed of the turret to see the Max, Min, and Current speed
  }

  public static double limit(double x, double upperLimit, double lowerLimit) {
    return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
     x;
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  } */
}
