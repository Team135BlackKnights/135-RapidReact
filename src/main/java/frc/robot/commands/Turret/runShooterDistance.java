// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Turret;

public class runShooterDistance extends CommandBase {
  /** Creates a new runShooter. */
  private final Turret turret;
  boolean isFinished = false;

  NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");
  NetworkTableEntry Ty = TurretLimelightTable.getEntry("ty"); 

  double angleGoalDegree, distance;
  double speedDesired, SkI, SkP, sError; //pid Numbers Shooter
  double hoodDesired, HkI, HkP, hError;

  
  
  public runShooterDistance(Turret subystem) {
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
    //<Distance>
    angleGoalDegree = 53 + Ty.getDouble(0.0);
        SmartDashboard.putNumber("Distance Rad", Math.toRadians(angleGoalDegree));
        distance = 161 / Math.tan(Math.toRadians(angleGoalDegree)); //distance in IN (hight of tape - hight of limelight) / tan(angle of limelight + angle of target)
    //</Distance>

    //<Shooter Speed>
    SmartDashboard.putNumber("RPM G", -turret.shooter.getRate() * 60);
    SmartDashboard.putNumber("RPM", -turret.shooter.getRate() * 60);
    SmartDashboard.putNumber("Shooter sError", sError);
    SmartDashboard.putNumber("Shooter Speed Desired", speedDesired * 10000);

    speedDesired = limit(-(RobotContainer.manipJoystick.getRawAxis(3) + 1) / 2, .8, 0); 
    sError = speedDesired + (speedDesired * .05) + (turret.shooter.getRate() * 60) / 10000;

    SkP = 13; //change when testing
    SkI = 5.5; //change when testing

    turret.LeftPower.set(outputs(sError * SkP, sError * SkI, speedDesired * 1.34, (speedDesired * 1.34) - speedDesired));
    turret.RightPower.set(-outputs(sError * SkP, sError * SkI, speedDesired * 1.34, (speedDesired * 1.34) - speedDesired));
    SmartDashboard.putNumber("Shooter Output", outputs(sError * SkP, sError * SkI, speedDesired * 1.34, (speedDesired * 1.34) - speedDesired));

    //</Shooter Speed>
    
    //<Turret Hight>
    hoodDesired = Math.floor(5.0895 * distance - 274.833);

    hError = turret.hoodHight.get() - hoodDesired;

    HkP = .004;
    HkI = .002;

    if (hError < 20)
      turret.hoodMotor.set(0);
    else if (turret.hoodHight.get() > 2500 && outputs(hError * HkP, hError * HkI, hoodDesired * 1.34, (hoodDesired * 1.34) - hoodDesired) > 0)
      turret.hoodMotor.set(0);
    else if (turret.hoodHight.get() < 50 && outputs(hError * HkP, hError * HkI, hoodDesired * 1.34, (hoodDesired * 1.34) - hoodDesired) < 0)
      turret.hoodMotor.set(0); //HEY DUMB DUMB, THIS IS WHATS BROKN, FIX THE LIMITS BEFORE TESTING MORE!
    else
    turret.hoodMotor.set(-outputs(hError * HkP, hError * HkI, hoodDesired * 1.34, (hoodDesired * 1.34) - hoodDesired));

    //</Turret Hight>
}

public static double limit(double x, double upperLimit, double lowerLimit) {
  return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
      x;
}

double outputs(double porOut, double iOut, double iBottom, double iTop) {
  if (porOut > iBottom && porOut < iTop) {
      SmartDashboard.putBoolean("I?", true);
      return limit(porOut + iOut, .85, 0);
  } else {
    SmartDashboard.putBoolean("I?", false);
      return limit(porOut, .85, 0);
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