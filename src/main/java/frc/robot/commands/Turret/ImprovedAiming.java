// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret.Aiming;

public class ImprovedAiming extends CommandBase {
  Aiming aiming;

  PIDController pidController = new PIDController(.08, 0, 0);
  NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");
  NetworkTableEntry Tx = TurretLimelightTable.getEntry("tx");
  NetworkTableEntry Tv = TurretLimelightTable.getEntry("tv");

  double setPoint;
  static boolean isCalibrated = false;

  public ImprovedAiming(Aiming m_Aiming) {
    addRequirements(m_Aiming);
    aiming = m_Aiming;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint = 75 / 2;
    SmartDashboard.putString("ImprovedAiming", "Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isCalibrated) {
      SmartDashboard.putString("ImprovedAiming", "Calibrating");
      if (aiming.LimitValue(aiming.LimitSwitch)) {
        aiming.angleMotor.set(-.4);
      } else {
        aiming.turretAngle.setPosition(0);
        isCalibrated = true;
      }
    } else {
      SmartDashboard.putString("ImprovedAiming", "Running");
      SmartDashboard.putNumber("AimingError", pidController.getPositionError());
      if (Tv.getDouble(0) == 1) {
        setPoint = aiming.turretAngle.getPosition() - Tx.getDouble(0.0); // set the desired pos to the current pos -
                                                                         // error
      } else {
        setPoint = 75 / 2; //else if cannot see change set point to default
      }
      
      pidController.calculate(aiming.turretAngle.getPosition(), setPoint); //store calculation of error
      if (Math.abs(pidController.getPositionError()) < .38) { // if the error is negliable, stop the command
        aiming.angleMotor.set(0);
      } else if (setPoint < 2 || setPoint > 78) { // if the setPoint is past the limts, don't move
        aiming.angleMotor.set(0);
      } else {
        aiming.angleMotor.set(pidController.calculate(aiming.turretAngle.getPosition(), setPoint)); //free to move where you want to 
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
