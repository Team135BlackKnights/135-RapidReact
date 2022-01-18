// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;



public class runIntake extends CommandBase {

  public final frc.robot.subsystems.Intake.Intake intake;
  public runIntake(frc.robot.subsystems.Intake.Intake subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.rightTrigger.get())
    {
      intake.LeftIntake.set(1);
      intake.RightIntake.set(1);
    }
    else 
    {
    
      intake.LeftIntake.set(0);
      intake.RightIntake.set(0);
    }
  
    if(RobotContainer.Button7.get())
    {
      intake.LeftIntake.set(-1);
      intake.RightIntake.set(-1);

    }

    else 

    {
      intake.LeftIntake.set(0);
      intake.RightIntake.set(0);
    } }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
