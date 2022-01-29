// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.Drive;

public class encoderDrive extends CommandBase {
  Drive drive;
  double Desired, lastOutput, kI, kP, porOut, error, iOut, iTop, iBottom, encodervalue;
  public boolean isFinished;
  /** Creates a new EncoderDrive. */
  public encoderDrive(Drive subsystem, double _inchesdesired) {
    Desired = _inchesdesired;
    drive = subsystem; //set drive as drivel
    addRequirements(drive);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    iTop = 0;
    iBottom = 0;
    porOut = 0;
    iOut = 0;
    kP = 0;
    kI = 0;
    Desired = 0;
    isFinished = false;
    drive.resetEncoders();
    
    SmartDashboard.putBoolean("encoder drive", true);

    encodervalue = (drive.e_BackLeft.getPosition() + drive.e_BackRight.getPosition() + drive.e_FrontRight.getPosition() + drive.e_FrontLeft.getPosition())/4 ;
   
   // encodervalue 
    
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (Math.abs(error) < 3) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
