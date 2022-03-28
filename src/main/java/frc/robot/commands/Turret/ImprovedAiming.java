// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret.Aiming;

public class ImprovedAiming extends CommandBase {
  Aiming aiming;
  boolean isFinished = false;

  PIDController pidController = new PIDController(.037, .01, .001);
  NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");
  NetworkTableEntry Tx = TurretLimelightTable.getEntry("tx");
  NetworkTableEntry Tv = TurretLimelightTable.getEntry("tv");

  double setPoint;

  public ImprovedAiming(Aiming m_Aiming) {
    addRequirements(m_Aiming);
    aiming = m_Aiming;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (aiming.Thresholding){
      isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(pidController.getPositionError()) < .5){ //if the error is negliable, stop the command
      isFinished = true;
    }

    setPoint = aiming.turretAngle.getPosition() - Tx.getDouble(0.0); //set the desired pos to the current pos - the distance to the target's center4
    
    if (setPoint < 5 || setPoint > 75){ //if the setPoint is past the limts, don't move
      aiming.angleMotor.set(0);
    } else if (Tv.getDouble(0.0) == 1){ //else if there is a target, move
      aiming.angleMotor.set(pidController.calculate(aiming.turretAngle.getPosition(), setPoint));
    }
    else {
      aiming.angleMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    aiming.angleMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
