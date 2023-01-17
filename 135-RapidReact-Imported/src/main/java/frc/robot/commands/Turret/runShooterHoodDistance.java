// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.Turret;

public class runShooterHoodDistance extends CommandBase {
  private final Turret turret;
  boolean isFinished = false;
  NetworkTable TurretLimelightTable = NetworkTableInstance.getDefault().getTable("limelight-turret");
  NetworkTableEntry Ty = TurretLimelightTable.getEntry("ty");
  NetworkTableEntry Tv = TurretLimelightTable.getEntry("tv");

  double angleGoalDegree, distance;

  double shooterOn, speedDesired, hoodDesired;

  PIDController speedPIDController = new PIDController(.000005, .000002, .000005);
  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, .000134); // 0001375

  PIDController HoodPIDController = new PIDController(.001, .005, 0);

  public runShooterHoodDistance(Turret m_Turret) {
    turret = m_Turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterOn = 0;

    speedPIDController.setIntegratorRange(-.02, .02);
    speedPIDController.enableContinuousInput(800, 6000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // <Distance>
    angleGoalDegree = 40 + Ty.getDouble(0.0);
    distance = (79 /* * 1.4 */) / Math.tan(Math.toRadians(angleGoalDegree));
    SmartDashboard.putNumber("Distance To Target", distance);
    // </Distance>

    //if (RobotContainer.manipButton3.get()) {
    if (RobotContainer.manipController.getAButtonPressed()) {
      shooterOn = 1;
    //} else if (RobotContainer.manipButton4.get()) {
    } else if (RobotContainer.manipController.getBButtonPressed()) {
      shooterOn = 0;
    }

    speedDesired = (5.625 * distance) + 3693.75;

    speedDesired *= shooterOn * Tv.getDouble(0.0);

    // <Ranges>
    if (Tv.getDouble(0) == 0 || distance < 50) { // if firing blind set power flat
      hoodDesired = 1000;
    } else if (distance < 100) {
      hoodDesired = calcPercent(50, 100, 1500, 2000, distance);
    } else if (distance < 150) {
      hoodDesired = calcPercent(100, 150, 2500, 2800, distance);
    } else if (distance < 175) {
      hoodDesired = calcPercent(150, 175, 2800, 2800, distance);
    } else if (distance < 200) {
      hoodDesired = calcPercent(175, 200, 3000, 3600, distance);
    } else if (distance < 225) {
      hoodDesired = calcPercent(200, 225, 3400, 3400, distance);
    } else if (distance < 260) {
      hoodDesired = calcPercent(226, 260, 3400, 3400, distance);
    }

    SmartDashboard.putNumber("HoodDesired", hoodDesired);
    SmartDashboard.putNumber("Hood Error", HoodPIDController.getPositionError());
    // </Ranges>

    SmartDashboard.putNumber("HoodOutput", HoodPIDController.calculate(-turret.hoodHight.getPosition() * 35.7, hoodDesired));
    turret.hoodMotor.set(-HoodPIDController.calculate(-turret.hoodHight.getPosition() * 35.7, hoodDesired));

    if (speedDesired == 0) {
      turret.LeftPower.set(0);
    } else {
      turret.LeftPower.set(feedForward.calculate(speedDesired)
          + speedPIDController.calculate(turret.shooter.getVelocity(), speedDesired));
    }
  }

  public static double calcPercent(double minDistance, double maxDistance, double maxOutput, double minOutput,
      double distance) {
    double distanceRange = maxDistance - minDistance;
    double percent = (-minDistance + distance) / distanceRange;
    return (((maxOutput - minOutput) * percent) + minOutput);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
