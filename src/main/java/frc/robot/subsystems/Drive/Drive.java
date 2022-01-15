// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drive extends SubsystemBase {
  public DifferentialDrive tank;
  public AHRS navx;
  CANSparkMax FrontLeft = new CANSparkMax(RobotMap.Drive.FL_ID, MotorType.kBrushless);
  CANSparkMax FrontRight = new CANSparkMax(RobotMap.Drive.FR_ID, MotorType.kBrushless);
  CANSparkMax BackLeft = new CANSparkMax(RobotMap.Drive.BL_ID, MotorType.kBrushless);
  CANSparkMax BackRight = new CANSparkMax(RobotMap.Drive.BR_ID, MotorType.kBrushless);

  private RelativeEncoder e_FrontLeft, e_FrontRight, e_BackLeft, e_BackRight;

  /** Creates a new Drive. */
  public Drive() {
    try{ //put all motor config code in here
    FrontLeft.enableVoltageCompensation(12);
    FrontRight.enableVoltageCompensation(12);
    BackLeft.enableVoltageCompensation(12);
    BackRight.enableVoltageCompensation(12); 
    
    e_FrontLeft = FrontLeft.getEncoder();
    e_FrontRight = FrontRight.getEncoder();
    e_BackLeft = BackLeft.getEncoder();
    e_BackRight = BackRight.getEncoder();
    }
    finally{ //free up the memory after the moters are configered
      FrontLeft.close();
      FrontRight.close();
      BackLeft.close();
      BackRight.close();
    }

    MotorControllerGroup left = new MotorControllerGroup(FrontLeft, BackLeft);
    MotorController right = new MotorControllerGroup(FrontRight, BackRight);

    tank = new DifferentialDrive(left, right);

    // Declares a new Navx and immediately sets it to 0
    navx = new AHRS(RobotMap.Drive.navXPort);
  }

  public void tankDrive(double left, double right){
    tank.tankDrive(left, right);
  }

  public void resetEncoders() {
    e_FrontLeft.setPosition(0);
    e_FrontRight.setPosition(0);
    e_BackLeft.setPosition(0);
    e_BackRight.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}