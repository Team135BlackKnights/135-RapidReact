// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveRobot;

public class encoderDrive extends CommandBase {
  DriveRobot drive;
  double Desired, lastOutput, kI, kP, porOut, error, iOut, iTop, iBottom, encodervalue;
  public boolean isFinished;
  /** Creates a new EncoderDrive. */
  public encoderDrive(DriveRobot subsystem, double _inchesdesired) {
    Desired = _inchesdesired;
    drive = subsystem; //set drive as drivel
    addRequirements(drive);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    iTop = Desired * .34;
    iBottom = Desired - (Desired * 1.34);
    kP = 0; //change when testing
    kI = 0; //change when testing
    isFinished = false;
    drive.resetEncoders();
    
    SmartDashboard.putBoolean("encoder drive", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encodervalue = ((drive.LeftSide.getDistance() + drive.RightSide.getDistance()) /2); //average of dr
  
    encodervalue = encodervalue / 42;

    encodervalue *= Math.PI * 6; 
    
    error = Desired - encodervalue;

    porOut = error * kP;
    iOut = error * kI;
    
    if (limit(outputs(), .40, -.40) < .07 && limit(outputs(), .40, -.40) > 0) {
        porOut = .07;
    } else if (limit(outputs(), .40, -.40) > -.07 && limit(outputs(), .40, -.40) < 0) {
        porOut = -.07;
    }

    drive.tankDrive(outputs(), outputs());
        
    if (Math.abs(error) < 1) {
      isFinished = true;
    }
  }

  double outputs() {
    if (porOut > iBottom && porOut < iTop) {
        return limit(porOut + iOut, .4, -.4);
    } else {
        return limit(porOut, .4, -.4);
    }
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
    return isFinished();
  }
}
