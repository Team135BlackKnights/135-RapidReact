// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveRobot;

public class tankDrive extends CommandBase {
  private final DriveRobot drive;
  /** Creates a new tankDrive. */
  public tankDrive(DriveRobot subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  
@Override
public void initialize() {
 // drive.navx.calibrate();
  drive.resetEncoders();
}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = -RobotContainer.leftJoystick.getRawAxis(RobotMap.KOI.VERTICAL_AXIS) * (-RobotContainer.leftJoystick.getRawAxis(3) + 1) / 2;
    double rightSpeed = RobotContainer.rightJoystick.getRawAxis(RobotMap.KOI.VERTICAL_AXIS) * (-RobotContainer.rightJoystick.getRawAxis(3) + 1) / 2;  

    drive.tankDrive(-leftSpeed, -rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
