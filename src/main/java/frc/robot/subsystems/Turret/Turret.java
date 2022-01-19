// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  CANSparkMax LeftPower = new CANSparkMax(RobotMap.Turret.PL_ID, MotorType.kBrushless);
  CANSparkMax RightPower = new CANSparkMax(RobotMap.Turret.PR_ID, MotorType.kBrushless);

  CANSparkMax angleMotor = new CANSparkMax(RobotMap.Turret.R_ID,MotorType.kBrushless);
  
  public RelativeEncoder shooter, turretAngle; 

  public Turret() {
    try {
      LeftPower.enableVoltageCompensation(12);
      RightPower.enableVoltageCompensation(12);
      angleMotor.enableVoltageCompensation(12);

      turretAngle = angleMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
      shooter = LeftPower.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    } 
    finally{
      LeftPower.close();
      RightPower.close();
      angleMotor.close();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double speed){
    LeftPower.set(speed);
    RightPower.set(-speed);
  }

  public void resetEncoders() {
    shooter.setPosition(0);
  }
}
