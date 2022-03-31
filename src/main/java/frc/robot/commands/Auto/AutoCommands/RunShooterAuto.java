// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret.Turret;

public class RunShooterAuto extends CommandBase {

  private final Turret turret;
    boolean isFinished = false;
    NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");
    NetworkTableEntry Ty = TurretLimelightTable.getEntry("ty");
    NetworkTableEntry Tv = TurretLimelightTable.getEntry("tv");

    double angleGoalDegree, distance;
    double speedDesired, SkI, SkP, sError, x, shooterOn; // pid Numbers Shooter
    double hoodDesired, HkI, HkP, hError;

    PIDController pidController = new PIDController(0.00005, 0, .000025);
    SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0, .0001);


  public RunShooterAuto(Turret subystem) {
    addRequirements(subystem);
    turret = subystem;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setIntegratorRange(-.02, .02);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // <Distance>
     angleGoalDegree = 53 + Ty.getDouble(0.0);
     distance = 160.5 / Math.tan(Math.toRadians(angleGoalDegree)); // distance in IN (hight of tape - hight of
                                                                 // limelight) / tan(angle of limelight + angle of
                                                                 // target)
     SmartDashboard.putNumber("Distance To Target", distance);
     // </Distance>
     SmartDashboard.putNumber("RPM G", turret.shooter.getVelocity());
     SmartDashboard.putNumber("RPM", turret.shooter.getVelocity());
     SmartDashboard.putNumber("Shooter sError", pidController.getVelocityError());


     if (Tv.getDouble(0) == 0) { // if firing blind set power flat
      speedDesired = 3800;
  } else if (distance < 75) {
      speedDesired = calcPercent(0, 75, 4120, 3700, distance);    //(minimum distance, max distance, max power, minumum power)
  } else if (distance < 100) {
      speedDesired = calcPercent(75, 100, 4250, 4120, distance);
  } else if (distance < 145) {
      speedDesired = calcPercent(100, 145, 4600, 4250, distance);
  } else if (distance < 175) {
      speedDesired = calcPercent(145, 175, 4900, 4600, distance);
  } else if (distance < 200) {
      speedDesired = calcPercent(175, 200, 4775, 4515, distance);
  } else {
      speedDesired = ((-0.0129955 * Math.pow(distance, 2) + (13.834 * distance) + 2627.57)); // decreased c by     //follow formula as backup
                                                                                             // 1550
  }
  // </Ranges> 
  turret.LeftPower.set(pidController.calculate(turret.shooter.getVelocity(), speedDesired) + FeedForward.calculate(speedDesired));

   // <Turret Hight>
   hoodDesired = Math.floor((.00835611 * Math.pow(distance, 2)) + (1.98118 * distance) + 1908.53); 
   // a decreased by 0
   // c increased by 200
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
