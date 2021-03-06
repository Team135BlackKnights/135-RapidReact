// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Turret;

public class runShooter extends CommandBase {
    /** Creates a new runShooter. */
    private final Turret turret;
    boolean isFinished = false;

    double desired, lastOutput, kI, kP, porOut, error, iOut, iTop, iBottom; //pid Numbers
    double minSpeed, currentSpeed, maxSpeed; //debug numbs

    public runShooter(Turret subystem) {
        addRequirements(subystem);
        turret = subystem;
    }

    // Called when the command is initially scheduled
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Joysticks", limit(-RobotContainer.leftJoystick.getRawAxis(3), .8, 0));
        SmartDashboard.putNumber("RPM G", turret.shooter.getRate() * 60);
        SmartDashboard.putNumber("RPM", turret.shooter.getRate() * 60);

        desired = limit(-RobotContainer.leftJoystick.getRawAxis(3), .8, 0); //change this num with testing
        error = desired - (turret.shooter.getRate() * 60) / 10000;

        iTop = desired * 1.34;
        iBottom = desired - (desired * 1.34);
        kP = 3; //change when testing
        kI = -1.3; //change when testing

        porOut = error * kP;
        iOut = error * kI;

        turret.LeftPower.set(outputs());
        turret.RightPower.set(-outputs());
        SmartDashboard.putNumber("OutPut", outputs());
    }

    public static double limit(double x, double upperLimit, double lowerLimit) {
        return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
            x;
    }

    double outputs() {
        if (porOut > iBottom && porOut < iTop) {
            return limit(porOut + iOut, .8, 0);
        } else {
            return limit(porOut, .8, 0);
        }
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}