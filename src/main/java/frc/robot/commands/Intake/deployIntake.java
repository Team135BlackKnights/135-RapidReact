// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class deployIntake extends CommandBase {
  // Creates a new deployIntake. 
  public final Intake intake;
  boolean isFinished = false;
  public deployIntake(Intake subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Intake", intake.Solenoid.get().toString());

    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.Solenoid.get() == Value.kForward ){
      intake.Solenoid.set(Value.kReverse);
    } else if (intake.Solenoid.get() == Value.kReverse) {
      intake.Solenoid.set(Value.kForward);
    } else {
      intake.Solenoid.set(Value.kReverse);
    }
    SmartDashboard.putString("Intake", intake.Solenoid.get().toString());
    isFinished = true;
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
