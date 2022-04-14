// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveRobot;

public class TimeDrive extends CommandBase {
  Timer timer = new Timer();
  DriveRobot drive;
  boolean isFinished = false;
  double DriveTime;
  
  public TimeDrive(DriveRobot m_drive, double i) {
    drive = m_drive;
    DriveTime = i;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.tankDrive(.5, -.5);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto Timer", Timer.getMatchTime());
    drive.tankDrive(.5, -.5);

    if (timer.get() > DriveTime)
    {
      drive.tankDrive(0, 0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished; 
  }
}
