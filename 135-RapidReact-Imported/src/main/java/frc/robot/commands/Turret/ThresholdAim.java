// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret.Aiming;

public class ThresholdAim extends CommandBase {
  Aiming aiming;

  boolean isFinished = false, runOnce;
  public ThresholdAim(Aiming m_aiming) {
    aiming = m_aiming;
    addRequirements(aiming);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    runOnce = Preferences.getBoolean("ThresholdRunOnce", false);
    isFinished = !runOnce;
    runOnce = true;
    Preferences.setBoolean("ThresholdRunOnce", runOnce);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!aiming.LimitValue(aiming.LimitSwitch)){
      aiming.angleMotor.set(.4);
    } else {
      isFinished = true;
    }
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
