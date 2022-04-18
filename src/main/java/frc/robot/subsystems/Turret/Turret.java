// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public CANSparkMax LeftPower = new CANSparkMax(RobotMap.Turret.PL_ID, MotorType.kBrushless);
  public CANSparkMax RightPower = new CANSparkMax(RobotMap.Turret.PR_ID, MotorType.kBrushless);

  public CANSparkMax hoodMotor = new CANSparkMax(RobotMap.Turret.HA_ID, MotorType.kBrushless);
  
  public RelativeEncoder shooter, hoodHight;

  public Turret() {
    shooter =     LeftPower.getEncoder();
    hoodHight =   hoodMotor.getEncoder();

    LeftPower.setInverted(true);
    LeftPower.setIdleMode(IdleMode.kCoast);
    LeftPower.enableVoltageCompensation(12);

    RightPower.follow(LeftPower, true); // follow the left motor, reverse direction
    RightPower.setIdleMode(IdleMode.kCoast);
    RightPower.enableVoltageCompensation(12);

    hoodMotor.setSmartCurrentLimit(6);
    hoodMotor.setIdleMode(IdleMode.kBrake);

    shooter.setVelocityConversionFactor(1.3333333333);

    LeftPower.burnFlash();
    RightPower.burnFlash();
    hoodMotor.burnFlash();
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("HoodHightRaw", -hoodHight.getPosition() * 37.5);
    SmartDashboard.putNumber("HoodOutput", hoodMotor.get());
    SmartDashboard.putNumber("Shooter Voltage ", LeftPower.getAppliedOutput());
    SmartDashboard.putNumber("ShooterSpeedRaw", LeftPower.get());
    SmartDashboard.putNumber("Shooter Temp", ((LeftPower.getMotorTemperature() + RightPower.getMotorTemperature()) / 2));
  }

  public void setPower(double speed){
    LeftPower.set(speed);
    RightPower.set(speed);
  }
  
  public void resetEncoders() {
    shooter.setPosition(0);
  }
}
