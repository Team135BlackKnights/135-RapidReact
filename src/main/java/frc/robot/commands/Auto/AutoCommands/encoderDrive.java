// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveRobot;

public class encoderDrive extends CommandBase {
  DriveRobot drive;
  double Desired, encodervalue;
  PIDController pidController = new PIDController(.008, .004, 0);
  public boolean isFinished;

  /** Creates a new EncoderDrive. */
  public encoderDrive(DriveRobot subsystem, double _inchesdesired) {
    Desired = _inchesdesired;
    drive = subsystem; // set drive as drivel
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    drive.resetEncoders();
    SmartDashboard.putBoolean("encoder drive", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encodervalue = (drive.lBack.getPosition() + drive.lFront.getPosition() - drive.rBack.getPosition()
        - drive.rFront.getPosition()) / 4; // average of dr

    SmartDashboard.putNumber("EncoderValueRAW", encodervalue);
        
    encodervalue = encodervalue / 42; //motor to gearbox

    encodervalue *= 12.94;

    encodervalue *= Math.PI * 6;

    SmartDashboard.putNumber("EncoderValue", encodervalue);

    drive.tankDrive(pidController.calculate(encodervalue, Desired), -pidController.calculate(encodervalue, Desired));

    if (Math.abs(pidController.getPositionError()) < 1) {
      isFinished = true;
    }
  }

  public static double limit(double x, double upperLimit, double lowerLimit) {
    return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit : x;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
