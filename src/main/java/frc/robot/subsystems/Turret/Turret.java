// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public CANSparkMax LeftPower = new CANSparkMax(RobotMap.Turret.PL_ID, MotorType.kBrushless);
  public CANSparkMax RightPower = new CANSparkMax(RobotMap.Turret.PR_ID, MotorType.kBrushless);

  public CANSparkMax hoodMotor = new CANSparkMax(RobotMap.Turret.HA_ID, MotorType.kBrushless);
  
  public Encoder shooter, hoodHight; 

  public DigitalInput LimitSwitch0, LimitSwitch1;

  public Turret() {
      shooter =     new Encoder(2, 3, false, Encoder.EncodingType.k4X);
      hoodHight =   new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  
  }
  public void setPower(double speed){
    LeftPower.set(speed);
    RightPower.set(-speed);
  }
  
  public void resetEncoders() {
    shooter.reset();
    hoodHight.reset();
  }
}
