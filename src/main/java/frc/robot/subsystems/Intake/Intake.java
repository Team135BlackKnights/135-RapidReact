// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {
    CANSparkMax LeftIntake = new CANSparkMax(RobotMap.Intake.LI_ID,MotorType.kBrushless);
    CANSparkMax RightIntake = new CANSparkMax(RobotMap.Intake.RI_ID, MotorType.kBrushless);
    Solenoid Solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.Intake.S1_ID);
    Solenoid Solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.Intake.S2_ID);

    LeftIntake.close();
    RightIntake.close();
    Solenoid1.close();
    Solenoid2.close();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
