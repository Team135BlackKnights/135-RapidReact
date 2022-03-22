// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveRobot extends SubsystemBase {
    public DifferentialDrive tank;
    Float x;
    public AHRS navx;
    CANSparkMax FrontLeft = new CANSparkMax(RobotMap.Drive.FL_ID, MotorType.kBrushless);
    CANSparkMax FrontRight = new CANSparkMax(RobotMap.Drive.FR_ID, MotorType.kBrushless);
    CANSparkMax BackLeft = new CANSparkMax(RobotMap.Drive.BL_ID, MotorType.kBrushless);
    CANSparkMax BackRight = new CANSparkMax(RobotMap.Drive.BR_ID, MotorType.kBrushless);
    public Encoder LeftSide, RightSide;
    public RelativeEncoder lFront, lBack, rFront, rBack;
    Timer timer = new Timer();

    /** Creates a new Drive. */
    public DriveRobot() {
        FrontLeft.enableVoltageCompensation(12);
        FrontRight.enableVoltageCompensation(12);
        BackLeft.enableVoltageCompensation(12);
        BackRight.enableVoltageCompensation(12);

        FrontLeft.setSmartCurrentLimit(80);
        FrontRight.setSmartCurrentLimit(80);
        BackLeft.setSmartCurrentLimit(80);
        BackRight.setSmartCurrentLimit(80); 

        FrontLeft.burnFlash();
        FrontRight.burnFlash();
        BackLeft.burnFlash();
        BackRight.burnFlash();
        
        lFront= FrontLeft.getEncoder();
        lBack = BackLeft.getEncoder();
        rFront= FrontRight.getEncoder();
        rBack = BackRight.getEncoder();
        
        MotorControllerGroup left = new MotorControllerGroup(FrontLeft, BackLeft);
        MotorControllerGroup right = new MotorControllerGroup(FrontRight, BackRight);

        tank = new DifferentialDrive(left, right);

        // Declares a new Navx and immediately sets it to 0
        navx = new AHRS(RobotMap.Drive.navXPort);
        navx.reset();
    }

    public void tankDrive(double left, double right) {
        tank.tankDrive(left, right);
    }

    public float navXCorrectOffset(){
        //math from here: https://www.chiefdelphi.com/t/off-centering-a-gyro/380703/7
        if (timer.get() != 0)
            // x = navx.getQuaternionX() / (float) timer.get(); //distance of navx per second
       // else 
         //    x = (float) 0;
        x = (float) (x * Math.pow(Math.PI, 2) * 5.67);          //Area of Navx
        return (float) (.2 * x * timer.get());                  //multiply by .2 and then convert to just degrees                    
    }

    public void resetEncoders() {
        LeftSide.reset();
        RightSide.reset();
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
       // if (navx.isRotating())
            //timer.start();
        //else {
           // timer.stop();
            timer.reset();
        //}

        SmartDashboard.putNumber("Left Power", (FrontLeft.get() + BackLeft.get()) / 2);
        SmartDashboard.putNumber("Right Power", (FrontRight.get() + BackRight.get()) / 2);
        //output side power
    }
}