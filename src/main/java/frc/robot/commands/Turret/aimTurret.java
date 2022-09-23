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

public class aimTurret extends CommandBase {
    /** Creates a new AutoAim. */
    Turret turret;
    NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");

    float Kp = .06f, Ki = -.02f;
    double EndPos, intergralTop, intergralBottom, proportional, intergral, error, desired, lastSeen;
    boolean isFinished = false, RunningSafety = false;

    NetworkTableEntry Ttx = TurretLimelightTable.getEntry("tx");
    NetworkTableEntry Tv = TurretLimelightTable.getEntry("tv");

    public aimTurret(Turret subsystem) {
        addRequirements(subsystem); //declare depencincy 
        turret = subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putNumber("VisableTarget", Tv.getDouble(0.0));
        turret.turretAngle.reset();
        SmartDashboard.putString("AutoAim:", "Initializing");

        while (turret.LimitSwitch1.get()) {
            SmartDashboard.putNumber("TurretAngleRaw", turret.turretAngle.get());
            turret.angleMotor.set(.1);
        }

        turret.turretAngle.reset();

        while (turret.LimitSwitch0.get()) {
            SmartDashboard.putNumber("TurretAngleRaw", turret.turretAngle.get());
            turret.angleMotor.set(-.1);
        }

        EndPos = turret.turretAngle.get();

        SafeCenter(true);

        turret.angleMotor.set(.2); //change for competition!!!!!!

        SmartDashboard.putNumber("EndPos", EndPos);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putString("AutoAim:", "Running");

        SmartDashboard.putNumber("VisableTarget", Tv.getDouble(0));
        error = Ttx.getDouble(0.0);
        SmartDashboard.putNumber("Current Error", error);


        if (lastSeen != Tv.getDouble(0.0) && Tv.getDouble(0.0) == 1) {
            intergralTop = Ttx.getDouble(0.0) * .34;
            intergralBottom = Ttx.getDouble(0.0) - (Ttx.getDouble(0.0) * 1.34);
        }

        if (Math.abs(error) < 1 && !RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Output", 0);
            turret.angleMotor.set(0);
            SmartDashboard.putBoolean("Error Finished", true); //if there error is negligable dont move
        } else if (turret.turretAngle.get() > EndPos - 1400 && !RunningSafety) {
            if (error < 0) {
                turret.angleMotor.set(0);
            } else {
                powerUpdate();
            }
        } else if (turret.turretAngle.get() < 1400 && !RunningSafety) {
            if (error > 0) {
                turret.angleMotor.set(0);
            } else {
                powerUpdate();
            }
        } else {
            powerUpdate();
        }



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

        lastSeen = Tv.getDouble(0.0);
    }

    //used to add a cap to motor speed
    public static double limit(double x, double upperLimit, double lowerLimit) {
        return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
            x;
    }

    public void powerUpdate() {
        if (Tv.getDouble(0) == 0) {
            if (turret.turretAngle.get() > EndPos - 1400) {
                turret.angleMotor.set(.4);
            }
            if (turret.turretAngle.get() < 1400) {
                turret.angleMotor.set(-.4);
            }
            SmartDashboard.putBoolean("Searching", true);
        } else {
            SmartDashboard.putBoolean("Searching", false);
        }

        if (error < intergralTop && error > intergralBottom && !RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Output", (limit(error * Ki + error * Kp, .8, -.8)));
            turret.angleMotor.set(limit(error * Ki + error * Kp, .8, -.8));
        // <Override>
        //if (RobotContainer.manipJoystick.getPOV() == 90) {
        if (RobotContainer.joystick.getPOV() == 90){
            aiming.angleMotor.set(.1);
        //} else if (RobotContainer.manipJoystick.getPOV() == 270) {
        } else if (RobotContainer.joystick.getPOV() == 270) {
            aiming.angleMotor.set(-.1);
        //} else if (RobotContainer.manipJoystick.getPOV() == 180) {
        } else if (RobotContainer.joystick.getXButtonPress()) {
            aiming.angleMotor.set(0);
        // </Override>
        } else if (error < intergralTop && error > intergralBottom && !RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Rotate Output", (limit(error * Ki + error * Kp, .8, -.8)));
            aiming.angleMotor.set(limit(error * Ki + error * Kp, .8, -.8));
        } else if (!RunningSafety && Tv.getDouble(0) == 1) {
            SmartDashboard.putNumber("Output", (limit(error * Kp, .8, -.8)));
            turret.angleMotor.set(limit(error * Kp, .8, -.8));
        }
    }

    public void SafeCenter(boolean Forward) {
        RunningSafety = true;
        Timer timer = new Timer();

        timer.start();

        while (timer.get() < 3) {
            if (Forward) turret.angleMotor.set(.3);
            else turret.angleMotor.set(-.3);

        }

        turret.angleMotor.set(0);

        timer.stop();
        RunningSafety = false;

        if (Forward) turret.angleMotor.set(.4);
        else turret.angleMotor.set(-.4);

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