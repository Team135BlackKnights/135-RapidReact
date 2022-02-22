// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret.Turret;

 class SafeCenterClass implements Runnable{
    boolean Forward;
    Turret turret;
    public SafeCenterClass(boolean m_Forward, Turret m_turret) {
        Forward = m_Forward;
        turret = m_turret;
    }

    @Override
    public void run(){
        Timer timer = new Timer();

        timer.start();

        while (timer.get() < 3) {
            if (Forward) turret.angleMotor.set(.3);
            else turret.angleMotor.set(-.3);

        }

        turret.angleMotor.set(0);

        timer.stop();
    }
}

public class aimTurret extends CommandBase {
    // Creates a new AutoAim. 
    Turret turret;
    NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");

    float Kp = .06f, Ki = -.02f;
    double EndPos, intergralTop, intergralBottom, proportional, intergral, error, desired, lastSeen, defaultThreadCount;
    public boolean isFinished = false, RunningSafety = false, thresholding = true, limit0Check = false, limit1Check = false;

    NetworkTableEntry Ttx = TurretLimelightTable.getEntry("tx"); 
    NetworkTableEntry Tv = TurretLimelightTable.getEntry("tv");

    public aimTurret(Turret subsystem) {
        addRequirements(subsystem); //declare depencincy 
        turret = subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putNumber("VisableTarget",Tv.getDouble(0.0));
        turret.turretAngle.reset();
        SmartDashboard.putString("AutoAim:", "Initializing");
        defaultThreadCount = Thread.activeCount();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("VisableTarget", Tv.getDouble(0));
        error = Ttx.getDouble(0.0);
        SmartDashboard.putNumber("Current Error", error);
        if (thresholding){
            if (!turret.LimitSwitch0.get())
                limit0Check = true;
                turret.turretAngle.reset();
            if (!turret.LimitSwitch1.get())
                limit1Check = true;

            if(!limit0Check && !limit1Check){
                turret.angleMotor.set(.1);
            }
            else if(limit0Check && !limit1Check){
                turret.angleMotor.set(-.1);
            }
            else if(limit0Check && limit1Check){
                EndPos = turret.turretAngle.get();
                SmartDashboard.putNumber("EndPos", EndPos);
                SafeCenter(true);
                thresholding = false;
            }
        }
        else{
            SmartDashboard.putString("AutoAim:", "Running");
        if (lastSeen != Tv.getDouble(0.0) && Tv.getDouble(0.0) == 1){
            intergralTop = Ttx.getDouble(0.0) * .34;
            intergralBottom = Ttx.getDouble(0.0) - (Ttx.getDouble(0.0) * 1.34);
        }

        if (Math.abs(error) < 1 && !RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Output", 0);
            turret.angleMotor.set(0);
            SmartDashboard.putBoolean("Error Finished", true); //if there error is negligable dont move
        } else if(turret.turretAngle.get() > EndPos - 1400 && !RunningSafety) {
            if (error < 0) {
                turret.angleMotor.set(0);
            }
            else {powerUpdate();}
        } else if(turret.turretAngle.get() < 1400 && !RunningSafety) {
            if (error > 0) {
                turret.angleMotor.set(0);
            }
            else {powerUpdate();}
        }
        else {powerUpdate();}

        if (!turret.LimitSwitch1.get()) {
            SmartDashboard.putBoolean("LIMIT TRIPPED", true);
            turret.angleMotor.set(0);
            SafeCenter(false);
        }
        if (!turret.LimitSwitch0.get()) {
            SmartDashboard.putBoolean("LIMIT TRIPPED", true);
            turret.angleMotor.set(0);
            SafeCenter(true);
        }
    }
        lastSeen = Tv.getDouble(0.0);
        if (Thread.activeCount() == defaultThreadCount){
            RunningSafety = false;
        }
    }

    //used to add a cap to motor speed
    public static double limit(double x, double upperLimit, double lowerLimit) {
        return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
            x;
    }

    public void powerUpdate(){
        if (Tv.getDouble(0) == 0){
            if(turret.turretAngle.get() > EndPos - 1400){
                turret.angleMotor.set(.4);
            }
            if (turret.turretAngle.get() < 1400){
                turret.angleMotor.set(-.4);
            }
            SmartDashboard.putBoolean("Searching", true);
        }
        else {SmartDashboard.putBoolean("Searching", false);}
        
        if (error < intergralTop && error > intergralBottom && !RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Output", (limit(error * Ki + error * Kp, .8, -.8)));
            turret.angleMotor.set(limit(error * Ki + error * Kp, .8, -.8));
        } else if (!RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Output", (limit(error * Kp, .8, -.8)));
            turret.angleMotor.set(limit(error * Kp, .8, -.8));
        }
    }

    public void SafeCenter(boolean Forward) {
        if (!RunningSafety){
        RunningSafety = true;
        Runnable r = new SafeCenterClass(Forward, turret);
        new Thread(r).start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AutoAim:", "Finished");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
