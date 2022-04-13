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
import edu.wpi.first.wpilibj.Timer;
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
    double speedDesired, SkI, SkP, sError, shooterOn; // pid Numbers Shooter
    double HkI, HkP, hError;
    float hoodDesired;
    Timer timer = new Timer();

    // original PID values with only one motor and old belt ratio (0.00005, 0,
    // .000025
    // original feedforward value

    PIDController pidController = new PIDController(.000005, .000002, .000005);
    SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0, .000134); // 0001375

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
        pidController.enableContinuousInput(800, 6000);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // <Distance>
        angleGoalDegree = 40 + Ty.getDouble(0.0);
        distance = (79 /* * 1.4 */) / Math.tan(Math.toRadians(angleGoalDegree)); // distance in IN (hight of tape -
                                                                                 // hight of
        // limelight) / tan(angle of limelight + angle of
        // target)
        SmartDashboard.putNumber("Distance To Target", distance);
        // </Distance>

        // <Shooter Speed>
        SmartDashboard.putNumber("RPM G", turret.shooter.getVelocity());
        SmartDashboard.putNumber("RPM", turret.shooter.getVelocity());
        SmartDashboard.putNumber("Shooter Error", pidController.getVelocityError());

        if (RobotContainer.manipButton3.get()) {
            shooterOn = 1;
        } else if (RobotContainer.manipButton4.get()) {
            shooterOn = 0;
        }

        // <Ranges>
        if (Tv.getDouble(0) == 0 || distance < 50) { // if firing blind set power flat
            speedDesired = 3500;
        } else if (distance < 100) {
            speedDesired = calcPercent(50, 100, 3750, 3450, distance);
        } else if (distance < 150) {
            speedDesired = calcPercent(100, 150, 4350, 3800, distance);
        } else if (distance < 175) {
            speedDesired = calcPercent(150, 175, 4650, 4300, distance);
        } else if (distance < 200) {
            speedDesired = calcPercent(175, 200, 4700, 4500, distance);
        } else if (distance < 225) {
            speedDesired = calcPercent(200, 225, 5100, 4800, distance);
        }else if (distance < 260) {
            speedDesired = calcPercent(226, 260, 5300, 5050, distance);
        }  else {
            speedDesired = ((-0.0129955 * Math.pow(distance, 2) + (13.834 * distance) + 3027.57)); // decreased c by
                                                                                                   // //follow formula
                                                                                                   // as backup
                                                                                                   // 1550
        }
        // </Ranges>

        speedDesired = speedDesired * shooterOn;

        if (speedDesired < 800) {
            speedDesired = 800;
        }

        if (Tv.getDouble(0.0) == 0 && shooterOn == 0) {
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

        SmartDashboard.putNumber("Shooter Speed Desired", speedDesired);

        if (speedDesired == 0) {
            turret.LeftPower.set(0);
            // } else if (currentColor == inverseColor){
            // turret.LeftPower.set(.2);
        } else {

            turret.LeftPower.set(FeedForward.calculate(speedDesired)
                    + pidController.calculate(turret.shooter.getVelocity(), speedDesired));
        }

        if (turret.shooter.getVelocity() < 850) {
            SmartDashboard.putString("Fire Status", "IDLE");
            timer.stop();
            timer.reset();
        } else if (Math.abs(turret.shooter.getVelocity()) < speedDesired + 50) {
            if (timer.get() == 0){
                timer.start();
            }
            SmartDashboard.putString("Fire Status", "UNSTABLE");
            if (timer.get() > 1.5) {
                SmartDashboard.putString("Fire Status", "READY");
                timer.stop();
            }
        } else if (turret.shooter.getVelocity() < speedDesired - 51) {
            SmartDashboard.putString("Fire Status", "SPIN UP");
        } else {
            SmartDashboard.putString("Fire Status", "UNSTABLE");
        }

        // </Shooter Speed>

        // <Turret Hight>
        // hoodDesired = Math.floor((-0.226112 * Math.pow(distance, 2)) + (67.5374 *
        // distance) + 2.39504);
        hoodDesired = Math.round(((12273299754007f * Math.pow(distance, 5)) / 165971415538252003680f)
                - ((1065637853052637f * Math.pow(distance, 4)) / 20746426942281500460f)
                + ((3514759159489241f * Math.pow(distance, 3)) / 260551672744508640f)
                - ((3143810159444512f * Math.pow(distance, 2)) / 1855673250651297f)
                + ((295470790575020762885f * distance) / 2766190258970866728f)
                + (1751294516999965475f / 16465418208159921f));


      //  hoodDesired -= 600;
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
                    1.34, (hoodDesired * 1.34) - hoodDesired), .5, -.5)); // power cannot exceed
            // .5
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
        timer.stop();
        timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}