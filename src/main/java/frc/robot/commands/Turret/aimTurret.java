// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class aimTurret extends CommandBase {
  frc.robot.subsystems.Turret.Turret turret;
  boolean isFinished = false;
  double error, desired, Kp, Ki, integralTop, integralBottom; //function Numbs
  /** Creates a new aimTurret. */

  public aimTurret(frc.robot.subsystems.Turret.Turret subsystem, double m_desired) {
    addRequirements(subsystem);
    turret = subsystem;
    desired = m_desired;
    // Use addRequirements() here to declare subsystem dependencies.
  }

 // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    error = desired - turret.turretAngle.getPosition();

    integralTop = desired * 1.34;
    integralBottom = desired - (desired * 1.34);

    Kp = .05;
    Ki = .01;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(error) < 3) {isFinished = true;}
    // check to see how many degrees off we are

    error = desired - turret.turretAngle.getPosition();
    //find the updated error

    if (error < integralTop && error > integralBottom){
      turret.setPower(limit(error * Ki + error * Kp, .89, -.89));
    }
    else {
     turret.setPower(limit(error * Kp, .89, -.89)); 
    }
    //
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
  }
}