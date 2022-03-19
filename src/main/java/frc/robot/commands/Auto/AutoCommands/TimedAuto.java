// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveRobot;

public class TimedAuto extends CommandBase {
  DriveRobot drive;
  Timer timer = new Timer();
  boolean isFinished = false;
  /** Creates a new TimedAuto. */
  public TimedAuto(DriveRobot m_drive) {
    drive = m_drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    drive.tankDrive(-.4, .4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto Timer", timer.get());
    if (timer.get() >= 5){
      drive.tankDrive(0, 0);
      timer.stop();
      isFinished = true;
    }
    else {
      drive.tankDrive(-.4, .4);

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
