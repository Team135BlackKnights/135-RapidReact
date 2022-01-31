// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
/*  CANSparkMax LeftPower = new CANSparkMax(RobotMap.Turret.PL_ID, MotorType.kBrushless);
  CANSparkMax RightPower = new CANSparkMax(RobotMap.Turret.PR_ID, MotorType.kBrushless);

  public CANSparkMax angleMotor = new CANSparkMax(RobotMap.Turret.R_ID, MotorType.kBrushless);
  
  public Encoder shooter, turretAngle; 

  public DigitalInput LimitSwitch0, LimitSwitch1;

  public Turret() {
    try {
      LeftPower.enableVoltageCompensation(12);
      RightPower.enableVoltageCompensation(12);
      angleMotor.enableVoltageCompensation(12);
      turretAngle = new Encoder(1, 0, false, Encoder.EncodingType.k4X);
      
  
  
      LimitSwitch0 = new DigitalInput(3);
      LimitSwitch1 = new DigitalInput(2);
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
    SmartDashboard.putNumber("TurretAngleRaw", turretAngle.get());

    SmartDashboard.putBoolean("Limit0", LimitSwitch0.get());
    SmartDashboard.putBoolean("Limit1", LimitSwitch1.get());
  }

  public void setPower(double speed){
    LeftPower.set(speed);
    RightPower.set(-speed);
  }

  public void resetEncoders() {
  }*/
}
