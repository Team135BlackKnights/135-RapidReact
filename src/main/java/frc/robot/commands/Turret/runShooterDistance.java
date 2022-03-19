// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

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
    double speedDesired, SkI, SkP, sError, x, shooterOn; //pid Numbers Shooter
    double hoodDesired, HkI, HkP, hError;

    boolean ballPersistant = false;

    Color RobotColor, inverseColor, lastSeenColor;
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
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red){
        RobotColor = kRedTarget;
        inverseColor = kBlueTarget;
      }
      if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
        RobotColor = kBlueTarget;
        inverseColor = kRedTarget;
      }    
      
      shooterOn = 0;
      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //<Color Selector>
        if (RobotContainer.manipButton7.get()){
          RobotColor = kRedTarget;
          inverseColor = kBlueTarget;
        }
        if (RobotContainer.manipButton8.get()){
          RobotColor = kBlueTarget;
          inverseColor = kRedTarget;
        }

        SmartDashboard.putString("Robot Color", RobotColor.toString());
        //<</Color Selector>

        //<Distance>
        angleGoalDegree = 53 + Ty.getDouble(0.0);
        SmartDashboard.putNumber("Distance Rad", Math.toRadians(angleGoalDegree));
        distance = 161 / Math.tan(Math.toRadians(angleGoalDegree)); //distance in IN (hight of tape - hight of limelight) / tan(angle of limelight + angle of target)
        SmartDashboard.putNumber("Distance To Target", distance);
        //</Distance>

        //<Shooter Speed>
        SmartDashboard.putNumber("RPM G", -turret.shooter.getRate() * 60);
        SmartDashboard.putNumber("RPM", -turret.shooter.getRate() * 60);
        SmartDashboard.putNumber("Shooter sError", sError);
        SmartDashboard.putNumber("Shooter Speed Desired", speedDesired);

        //speedDesired = limit((-RobotContainer.manipJoystick.getRawAxis(3) + 1) / 2, .8, 0);
        //SmartDashboard.putNumber("Shooter Speed Desired", speedDesired * 10000);

        if (RobotContainer.manipButton3.get()){
          shooterOn = 1;
        }
        else if (RobotContainer.manipButton4.get()){
          shooterOn = 0;
        }



        //<Ranges>
        if (distance < 75){
         speedDesired = calcPercent(0, 75, 4200, 3900, distance);
        } else if (distance < 100){ 
          speedDesired = calcPercent(75, 100, 4300, 4000, distance); 
        } else if (distance < 145){
          speedDesired = calcPercent(100, 145, 4450, 4200, distance);
        } else if(distance < 200){
          speedDesired = calcPercent(145, 200, 5400, 4500, distance);
        } else {
          speedDesired = ((-0.0129955 * Math.pow(distance, 2) + (13.834 * distance) + 3127.57)); //decreased by 350
        }
        //</Ranges>

        speedDesired = speedDesired * shooterOn;

        if (speedDesired < 800 || Tv.getDouble(0.0) == 0)
        {speedDesired = 800;}

        sError = (speedDesired + (speedDesired * .05) + (turret.shooter.getRate() * 60)) / 10000;

        SkP = 13; //change when testing
        SkI = 5.5; //change when testing

        ColorMatchResult match = m_colorMatcher.matchClosestColor(turret.colorSensor.getColor());

        if (match.color == kBlueTarget)
          SmartDashboard.putString("Ball Color", "Blue");
        else if (match.color == kRedTarget)
          SmartDashboard.putString("Ball Color", "Red");
        else  if (match.color == kGreenTarget)
         SmartDashboard.putString("Ball Color", "Nothing");
        else 
         SmartDashboard.putString("Ball Color", "Unknown");

        SmartDashboard.putString("Raw Ball Color", match.color.toString());

        if (lastSeenColor != match.color && lastSeenColor != kGreenTarget){
          x = 0;
          ballPersistant = true;
        }

        SmartDashboard.putNumber("X", x);

        if (x > 50)
        {
          ballPersistant = false;
        }

       // if (match.color == RobotColor || (ballPersistant == true && lastSeenColor == RobotColor)){

        turret.setPower(outputs(sError * SkP, sError * SkI, speedDesired * 1.34, (speedDesired * 1.34) - speedDesired));
         
        SmartDashboard.putString("FireReady?", "READY");
        SmartDashboard.putNumber("Shooter Output", outputs(sError * SkP, sError * SkI, speedDesired * 1.34, (speedDesired * 1.34) - speedDesired));

     /*   } else if(match.color == inverseColor || (ballPersistant == true && lastSeenColor == inverseColor)){
          turret.setPower(.1);
          SmartDashboard.putString("FireReady?", "WRONG COLOR");
          SmartDashboard.putNumber("Shooter Output", .1);
        } else {
          turret.setPower(0); 
          SmartDashboard.putString("FireReady?", "NO BALL");
          SmartDashboard.putNumber("Shooter Output", 0);
        }
 */
        if (ballPersistant = false)
          lastSeenColor = match.color;

        //</Shooter Speed>

        //<Turret Hight>
        //hoodDesired = Math.floor(19.26864 * distance - 110.50656); //decreased M by 615 | X increased by 5
        hoodDesired = Math.floor((-0.0297835 * Math.pow(distance, 2)) + (18.023 * distance) + 810.1803); //a decreased by .001 c increaded by 400
        SmartDashboard.putNumber("HoodDesired", hoodDesired);

        hError = turret.hoodHight.get() - hoodDesired;
        SmartDashboard.putNumber("HoodError", hError);

        HkP = .001;
        HkI = .005;

        if (Math.abs(hError) < 30) {
            turret.hoodMotor.set(0);
            SmartDashboard.putString("HoodMotorMode", "OnTarget");
        } else if (Tv.getDouble(0.0) == 0){
          turret.hoodMotor.set(0);
          SmartDashboard.putString("HoodMotorMode", "LostTarget");
        } else if (turret.hoodHight.get() > 2900 && outputs(hError * HkP, hError * HkI, hoodDesired * 1.34, (hoodDesired * 1.34) - hoodDesired) < 0) {
            turret.hoodMotor.set(0);
            SmartDashboard.putString("HoodMotorMode", "AtLimit");
        } else if (turret.hoodHight.get() < 950 && outputs(hError * HkP, hError * HkI, hoodDesired * 1.34, (hoodDesired * 1.34) - hoodDesired) > 0) {
            turret.hoodMotor.set(0);
            SmartDashboard.putString("HoodMotorMode", "AtLimit");
        } else {
           turret.hoodMotor.set(limit(outputs(hError * HkP, hError * HkI, hoodDesired * 1.34, (hoodDesired * 1.34) - hoodDesired), .5, -.5));
            SmartDashboard.putString("HoodMotorMode", "Ajusting");
        }

        //</Turret Hight>
    }

    public static double limit(double x, double upperLimit, double lowerLimit) {
        return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
            x;
    }

    double outputs(double porOut, double iOut, double iBottom, double iTop) {
        if (porOut > iBottom && porOut < iTop) {
            SmartDashboard.putBoolean("I?", true);
            return limit(porOut + iOut, .8, -.8);
        } else {
            SmartDashboard.putBoolean("I?", false);
            return limit(porOut, .85, -.8);
        }
    }

    public static double calcPercent(double minDistance, double maxDistance, double maxOutput, double minOutput, double distance){
      double distanceRange = maxDistance - minDistance;
      double percent = (-minDistance + distance) / distanceRange;
      return (((maxOutput - minOutput)*percent) + minOutput);
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