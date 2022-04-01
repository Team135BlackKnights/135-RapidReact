// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Turret;

public class runShooterDistance extends CommandBase {
    /** Creates a new runShooter. */
    private final Turret turret;
    boolean isFinished = false;
    NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");
    NetworkTableEntry Ty = TurretLimelightTable.getEntry("ty");
    NetworkTableEntry Tv = TurretLimelightTable.getEntry("tv");

    double angleGoalDegree, distance;
    double speedDesired, SkI, SkP, sError, x, shooterOn; // pid Numbers Shooter
    double hoodDesired, HkI, HkP, hError;

    // original PID values with only one motor and old belt ratio (0.00005, 0, .000025)
    // original feedforward value

    PIDController pidController = new PIDController(.000075, 0, 0);
    SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0, .0001375);

    boolean ballPersistant = false;

    Color RobotColor, inverseColor, lastSeenColor, currentColor;
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);

    ColorMatch m_colorMatcher = new ColorMatch();

    public runShooterDistance(Turret subystem) {
        addRequirements(subystem);
        turret = subystem;
    }

    // Called when the command is initially scheduled
    @Override
    public void initialize() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            RobotColor = kRedTarget;
            inverseColor = kBlueTarget;
        }
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            RobotColor = kBlueTarget;
            inverseColor = kRedTarget;
        }

        shooterOn = 0;
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);

        pidController.setIntegratorRange(-.02, .02);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // <Distance>
        angleGoalDegree = 40 + Ty.getDouble(0.0);
        distance = 79 / Math.tan(Math.toRadians(angleGoalDegree)); // distance in IN (hight of tape - hight of
                                                                    // limelight) / tan(angle of limelight + angle of
                                                                    // target)
        SmartDashboard.putNumber("Distance To Target", distance);
        // </Distance>

        // <Shooter Speed>
        SmartDashboard.putNumber("RPM G", turret.shooter.getVelocity());
        SmartDashboard.putNumber("RPM", turret.shooter.getVelocity());
        SmartDashboard.putNumber("Shooter sError", pidController.getVelocityError());

        if (RobotContainer.manipButton3.get()) {
            shooterOn = 1;
        } else if (RobotContainer.manipButton4.get()) {
            shooterOn = 0;
        }

        // <Ranges>
        if (Tv.getDouble(0) == 0) { // if firing blind set power flat
            speedDesired = 3800;
        } else if (distance < 75) {
            speedDesired = calcPercent(0, 75, 4000, 3550, distance);
        } else if (distance < 100) {
            speedDesired = calcPercent(75, 100, 4150, 4000, distance);
        } else if (distance < 150) {
            speedDesired = calcPercent(100, 150, 4570, 4200, distance);
        } else if (distance < 175) {
            speedDesired = calcPercent(150, 175, 4900, 4520, distance);
        } else if (distance < 200) {
            speedDesired = calcPercent(175, 200, 5075, 4900, distance);
        } else {
            speedDesired = ((-0.0129955 * Math.pow(distance, 2) + (13.834 * distance) + 2627.57)); // decreased c by     //follow formula as backup
                                                                                                   // 1550
        }
        // </Ranges>

        speedDesired = speedDesired * shooterOn;

        if (speedDesired < 800) {
            speedDesired = 800;
        }

        if (Tv.getDouble(0.0) == 0 && shooterOn == 0){
            speedDesired = 0;
        }

        ColorMatchResult match = m_colorMatcher.matchClosestColor(turret.colorSensor.getColor());
        if (match.confidence < 70) {
            SmartDashboard.putString("Ball Color", "No Ball");
            currentColor = null;
        } else if (match.color == kBlueTarget) {
            SmartDashboard.putString("Ball Color", "Blue");
            currentColor = match.color;
        } else if (match.color == kRedTarget) {
            SmartDashboard.putString("Ball Color", "Red");
            currentColor = match.color;
        } else {
            SmartDashboard.putString("Ball Color", "Unknown");
        }

        SmartDashboard.putString("Raw Ball Color", match.color.toString());

        if (lastSeenColor != currentColor && lastSeenColor != null) {
            x = 0;
            ballPersistant = true;
        }

        SmartDashboard.putNumber("X", x);

        if (x > 50) {
            ballPersistant = false;
        }

        // if (match.color == RobotColor || (ballPersistant == true && lastSeenColor ==
        // RobotColor)){
        SmartDashboard.putNumber("Shooter Speed Desired", speedDesired);
        SmartDashboard.putNumber("Shooter Speed Desired G", speedDesired);

        if (speedDesired == 0) {
            turret.LeftPower.set(0);
        } else {
            turret.LeftPower.set(pidController.calculate(turret.shooter.getVelocity(), speedDesired) + FeedForward.calculate(speedDesired));
        }

        if (turret.shooter.getVelocity() < 850){
            SmartDashboard.putString("Fire Status", "IDLE");
        } else if (!(turret.shooter.getVelocity() > speedDesired + 150)
                && !(turret.shooter.getVelocity() < speedDesired - 150)) {
            SmartDashboard.putString("Fire Status", "READY");
        } else if (turret.shooter.getVelocity() < speedDesired - 800) {
            SmartDashboard.putString("Fire Status", "SPIN UP");
        } else {
            SmartDashboard.putString("Fire Status", "UNSTABLE");
        }

        /*
         * } else if(currentColor == inverseColor || (ballPersistant == true &&
         * lastSeenColor == inverseColor)){
         * turret.setPower(.1);
         * SmartDashboard.putString("FireReady?", "WRONG COLOR");
         * SmartDashboard.putNumber("Shooter Output", .1);
         * } else {
         * turret.setPower(0);
         * SmartDashboard.putString("FireReady?", "NO BALL");
         * SmartDashboard.putNumber("Shooter Output", 0);
         * }
         */
        if (ballPersistant = false)
            lastSeenColor = match.color;

        // </Shooter Speed>

        // <Turret Hight>
        hoodDesired = Math.floor((-.0371293 * Math.pow(distance, 2)) + (16.8441 * distance) + 1296.05); 
        //hoodDesired = Math.floor((.470836 * Math.pow(distance, 1.39049)) + 1915.27); 

        SmartDashboard.putNumber("HoodDesired", hoodDesired);
        hError = -turret.hoodHight.getPosition() * 37.5 - hoodDesired;
        SmartDashboard.putNumber("HoodError", hError);

        HkP = .001;
        HkI = .005;

        if (Math.abs(hError) < 30) {
            turret.hoodMotor.set(0);
            SmartDashboard.putString("HoodMotorMode", "OnTarget");
        } else if (Tv.getDouble(0.0) == 0) {
            turret.hoodMotor.set(0);
            SmartDashboard.putString("HoodMotorMode", "LostTarget");
        } else if (-turret.hoodHight.getPosition() * 37.5 > 3400
                && outputs(hError * HkP, hError * HkI, hoodDesired * 1.34, (hoodDesired * 1.34) - hoodDesired) < 0) {
            turret.hoodMotor.set(0);
            SmartDashboard.putString("HoodMotorMode", "AtLimit");
        } else if (-turret.hoodHight.getPosition() * 37.5 < 400
                && outputs(hError * HkP, hError * HkI, hoodDesired * 1.34, (hoodDesired * 1.34) - hoodDesired) > 0) {
            turret.hoodMotor.set(0);
            SmartDashboard.putString("HoodMotorMode", "AtLimit");
        } else {
            turret.hoodMotor.set(limit(outputs(hError * HkP, hError * HkI, hoodDesired *
                     1.34, (hoodDesired * 1.34) - hoodDesired), .5, -.5)); //power cannot exceed .5 
            SmartDashboard.putString("HoodMotorMode", "Ajusting");
        }

        // </Turret Height>
    }

    public static double limit(double x, double upperLimit, double lowerLimit) {
        return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit : x;
    }

    double outputs(double porOut, double iOut, double iBottom, double iTop) {
        if (porOut > iBottom && porOut < iTop) {
            SmartDashboard.putBoolean("I?", true);
            return limit(porOut + iOut, .85, -.8);
        } else {
            SmartDashboard.putBoolean("I?", false);
            return limit(porOut, .85, -.8);
        }
    }

    public static double calcPercent(double minDistance, double maxDistance, double maxOutput, double minOutput,
            double distance) {
        double distanceRange = maxDistance - minDistance;
        double percent = (-minDistance + distance) / distanceRange;
        return (((maxOutput - minOutput) * percent) + minOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}