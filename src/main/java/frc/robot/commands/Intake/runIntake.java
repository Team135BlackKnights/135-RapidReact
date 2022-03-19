// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;



public class runIntake extends CommandBase {

  public final Intake intake;

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);

  Color RobotColor, inverseColor;


  ColorMatch m_colorMatcher = new ColorMatch();
  public runIntake(Intake subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
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

     // ColorMatchResult match = m_colorMatcher.matchClosestColor(intake.colorSensorV3.getColor());
    //</Color Selector>
    
    //<Intake>

        if (RobotContainer.rightTrigger.get())
        {
          intake.IntakeMotor.set(-.5);
        }

        if (RobotContainer.manipTrigger.get()) {
          intake.Feeder.set(.6);
        }
    //</Intake>

    //<Spit Out>
      if(RobotContainer.manipButton5.get() && !RobotContainer.rightTrigger.get())
      {
        intake.IntakeMotor.set(.4);
      }
    
      if (RobotContainer.manipButton6.get() && !RobotContainer.manipTrigger.get()) {
        intake.Feeder.set(-.8);
      }
    //</Spit Out>

    //<Shut Off>
      if (!RobotContainer.manipButton5.get() && !RobotContainer.rightTrigger.get()){
        intake.IntakeMotor.set(0);
      }

      if (!RobotContainer.manipButton6.get() && !RobotContainer.manipTrigger.get()){
        intake.Feeder.set(0);
      }
    //</Shut Off>
  }

    public void runCommand(boolean power) {
      if (power) {
      intake.IntakeMotor.set(.9);
      }
      else {
        intake.IntakeMotor.set(0);
      }

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
