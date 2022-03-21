// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public CANSparkMax LeftPower = new CANSparkMax(RobotMap.Turret.PL_ID, MotorType.kBrushless);
  public CANSparkMax hoodMotor = new CANSparkMax(RobotMap.Turret.HA_ID, MotorType.kBrushless);
  
  //public Encoder hoodHight; 
  public RelativeEncoder shooter;
  public Encoder hoodHeight;
  public ColorSensorV3 colorSensor = new ColorSensorV3(RobotMap.Intake.colorPort);

  public Turret() {
    //shooter =     new Encoder(4, 5, false, Encoder.EncodingType.k4X);
    shooter = LeftPower.getEncoder();
    hoodHeight = new Encoder(6, 7, true, Encoder.EncodingType.k4X);
    //hoodMotor.setSmartCurrentLimit(1);
    LeftPower.setSmartCurrentLimit(80);
    LeftPower.enableVoltageCompensation(12);

    LeftPower.burnFlash();
   // hoodMotor.burnFlash();
   hoodMotor.setIdleMode(IdleMode.kBrake);

   // shooter.set(.00048828125);
   hoodHeight.reset();
   shooter.setVelocityConversionFactor(2);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("HoodHightRaw", hoodHeight.get());
    SmartDashboard.putNumber("HoodOutput", hoodMotor.get());
    SmartDashboard.putNumber("ShooterMotorRaw", LeftPower.get());
    SmartDashboard.putNumber("Shooter Temp", (LeftPower.getMotorTemperature()));
  }

  public void setPower(double speed){
    LeftPower.set(speed);
    //RightPower.set(-speed);
  }
  
  public void resetEncoders() {
    shooter.setPosition(0);
    hoodHeight.reset();
  }
}
