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
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Aiming;

class SafeCenterClass implements Runnable {
    boolean Forward;
    Aiming aiming;

    public SafeCenterClass(boolean m_Forward, Aiming m_aiming) {
        Forward = m_Forward;
        aiming = m_aiming;
    }

    @Override
    public void run() {
        Timer timer = new Timer();

        timer.start();

        while (timer.get() < 1) {
            if (Forward)
                aiming.angleMotor.set(.6);
            else
                aiming.angleMotor.set(-.6);

        }
        timer.stop();
        aiming.angleMotor.set(0);
    }
}

public class aimTurret extends CommandBase {
    // Creates a new AutoAim.
    Aiming aiming;
    NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");

    float Kp = .037f, Ki = .01f;
    double EndPos = 75, intergralTop, intergralBottom, proportional, intergral, error, desired, lastSeen,
            defaultThreadCount, distance, angleGoalDegree;
    public boolean isFinished = false, RunningSafety = false, thresholding = true, limit0Check = false,
            limit1Check = true;

    NetworkTableEntry Ttx = TurretLimelightTable.getEntry("tx");
    NetworkTableEntry Tv = TurretLimelightTable.getEntry("tv");
    NetworkTableEntry Ty = TurretLimelightTable.getEntry("ty");

    public aimTurret(Aiming subsystem) {
        addRequirements(subsystem); // declare depencincy
        aiming = subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TurretLimelightTable.getEntry("ledMode").setNumber(0);
        SmartDashboard.putNumber("VisableTarget", Tv.getDouble(0.0));
        aiming.turretAngle.setPosition(0);
        thresholding = true;
        SmartDashboard.putString("AutoAim:", "Initializing");
        defaultThreadCount = Thread.activeCount();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("TY", Ty.getDouble(0.0));
        SmartDashboard.putNumber("ThreadCount", Thread.activeCount());
        SmartDashboard.putNumber("VisableTarget", Tv.getDouble(0));
        SmartDashboard.putBoolean("SafeCenter", RunningSafety);
        //<Light Controll>
        if (-RobotContainer.manipJoystick.getRawAxis(3) > 0){
            NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("ledMode").setNumber(3);
        } else if (-RobotContainer.manipJoystick.getRawAxis(3) < 0){
            NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("ledMode").setNumber(1);
        }
        //</Light Controll>

        error = Ttx.getDouble(0.0);
        SmartDashboard.putNumber("Rotate Current Error", error);

        // <Thresholding>
        if (thresholding) {
            if (!aiming.LimitValue(aiming.LimitSwitch0)) {
                limit0Check = true;
                aiming.turretAngle.setPosition(0);
            }
            if (!aiming.LimitValue(aiming.LimitSwitch1))
                limit1Check = true;

            if (!limit0Check) {
                aiming.angleMotor.set(.4);
            }
            /*
             * else if(limit0Check && !limit1Check){
             * aiming.angleMotor.set(-.2);
             * }
             */
            else if (limit0Check && limit1Check) {
                // EndPos = aiming.turretAngle.get();
                SmartDashboard.putNumber("EndPos", EndPos);
                SafeCenter(false);
                thresholding = false;
            }
        }
        // <Thresholding>

        else {
            // <Auto Run>
            SmartDashboard.putString("AutoAim:", "Running");
            if (lastSeen != Tv.getDouble(0.0) && Tv.getDouble(0.0) == 1) {
                intergralTop = Ttx.getDouble(0.0) * .34;
                intergralBottom = Ttx.getDouble(0.0) - (Ttx.getDouble(0.0) * 1.34);
            }

            if (RobotContainer.manipJoystick.getPOV() != -1) {
                powerUpdate();
            } else if (!aiming.LimitValue(aiming.LimitSwitch0)) {
                aiming.angleMotor.set(0);
            } else if (Math.abs(error) < .4 && !RunningSafety && Tv.getDouble(0) == 1) {
                SmartDashboard.putNumber("Rotate Output", 0);
                aiming.angleMotor.set(0);
                SmartDashboard.putBoolean("Error Finished", true); // if there error is negligable dont move
            } else if (-aiming.turretAngle.getPosition() > EndPos - 6 && !RunningSafety) {
                if (error < 0) {
                    aiming.angleMotor.set(0);
                } else {
                    powerUpdate();
                }
            } else if (-aiming.turretAngle.getPosition() < 6 && !RunningSafety) {
                if (error > 0) {
                    aiming.angleMotor.set(0);
                } else {
                    powerUpdate();
                }
            } else {
                powerUpdate();
                SmartDashboard.putBoolean("Error Finished", false);
            }
            // </Auto Run>
        }

        lastSeen = Tv.getDouble(0.0);

        if (Thread.activeCount() == defaultThreadCount) {
            RunningSafety = false;
        }
    }

    // used to add a cap to motor speed
    public static double limit(double x, double upperLimit, double lowerLimit) {
        return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit : x;
    }

    public void powerUpdate() {
        // <Override>
        if (RobotContainer.manipJoystick.getPOV() == 90) {
            aiming.angleMotor.set(.1);
        } else if (RobotContainer.manipJoystick.getPOV() == 270) {
            aiming.angleMotor.set(-.1);
        } else if (RobotContainer.manipJoystick.getPOV() == 180) {
            aiming.angleMotor.set(0);
        // </Override>
        } else if (error < intergralTop && error > intergralBottom && !RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Rotate Output", (limit(error * Ki + error * Kp, .8, -.8)));
            aiming.angleMotor.set(limit(error * Ki + error * Kp, .8, -.8));
        } else if (!RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Rotate Output", (limit(error * Kp, .8, -.8)));
            aiming.angleMotor.set(limit(error * Kp, .8, -.8));
        }
    }

    public void SafeCenter(boolean Forward) {
        if (!RunningSafety) {
            RunningSafety = true;
            Runnable r = new SafeCenterClass(Forward, aiming);
            new Thread(r).start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        thresholding = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
