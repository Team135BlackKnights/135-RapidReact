// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveRobot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret.Aiming;
import frc.robot.subsystems.Turret.Turret;

public class TimedAuto extends CommandBase {
  DriveRobot drive;
  Turret turret;
  Intake intake;
  Aiming aiming;
  Timer timer = new Timer();
  boolean isFinished = false;
  /** Creates a new TimedAuto. */
  public TimedAuto(DriveRobot m_drive, Turret m_turret, Intake m_intake, Aiming m_aiming) {
    drive = m_drive;
    turret = m_turret;
    aiming = m_aiming;
    intake = m_intake;
    addRequirements(drive, turret, intake, aiming);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
   //drive.tankDrive(-.4, .4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto Timer", timer.get());
    if (timer.get() >= 1.6){
      drive.tankDrive(0, 0);
      if (timer.get() > 3){
        turret.LeftPower.set(.4);
        turret.hoodMotor.set(.2);
        if (timer.get() > 5){
        turret.hoodMotor.set(0);
        intake.Feeder.set(.8);
        }
      }
    }
    else {
    //drive.tankDrive(-.4, .4);
    }
    if (timer.get() > 8){
      timer.stop();
      turret.LeftPower.set(0);;
      intake.Feeder.set(0);
    }
    if (timer.get() > 15){
      isFinished = true;
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
