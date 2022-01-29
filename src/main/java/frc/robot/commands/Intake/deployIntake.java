// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.Intake;

public class deployIntake extends CommandBase {
  // Creates a new deployIntake. 
  public final Intake intake;
  public deployIntake(Intake subsystem) {
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
    
    if(RobotContainer.Button8.get()){
     intake.Solenoid1.set(true);   //set solenoids to one position
    }
    
    else {
      //set solenoids to other position
      intake.Solenoid1.set(false);

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
