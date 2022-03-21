// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

/** Creates a new Intake. */
  //public DoubleSolenoid Solenoid1, Solenoid2;

  public CANSparkMax Feeder = new CANSparkMax(RobotMap.Intake.FM_ID, MotorType.kBrushless);
  public CANSparkMax IntakeMotor = new CANSparkMax(RobotMap.Intake.InM_ID, MotorType.kBrushless);

  //public ColorSensorV3 colorSensorV3 = new ColorSensorV3(RobotMap.Intake.colorPort);
  public DoubleSolenoid Solenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 0, 1);

  public Intake() {
      Feeder.setSmartCurrentLimit(5);
      IntakeMotor.setSmartCurrentLimit(50);

      Feeder.burnFlash();
      IntakeMotor.burnFlash();
     }

  @Override
  public void periodic(){}
      // This method will be called once per scheduler run
  }
