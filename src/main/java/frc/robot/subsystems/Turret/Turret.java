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
  
  private RelativeEncoder shooter; 

  public Turret() {
    try {
      LeftPower.enableVoltageCompensation(12);
      RightPower.enableVoltageCompensation(12);

      shooter = LeftPower.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    } 
    finally{
      LeftPower.close();
      RightPower.close();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
