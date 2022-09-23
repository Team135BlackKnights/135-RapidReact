// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.Intake;



public class runIntake extends CommandBase {

    public final Intake intake;
    public runIntake(Intake subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        intake = subsystem;
        addRequirements(subsystem);
    }

        //if (RobotContainer.rightTrigger.get()) {
        if (RobotContainer.joystick.getLeftTriggerAxis() > 0) {
          intake.IntakeMotor.set(-.5);
        }

        //if (RobotContainer.manipTrigger.get()) {
        if (RobotContainer.joystick.getRightTriggerAxis() > 0) {
          intake.Feeder.set(.6);
        }
    //</Intake>

    //<Spit Out>
      //if(RobotContainer.manipButton9.get()) {
      if (RobotContainer.joystick.getLeftTriggerAxis() > 0 && RobotContainer.joystick.getButtonBPressed()) {
        intake.IntakeMotor.set(.4);
      }
    
      //if (RobotContainer.manipButton8.get()) {
      if (RobotContainer.joystick.getRightTriggerAxis() > 0 && RobotContainer.joystick.getButtonBPressed()) {
        intake.Feeder.set(-.8);
      }
    //</Spit Out>

    //<Shut Off>
      //if (!RobotContainer.manipButton5.get() && !RobotContainer.rightTrigger.get()){
      if (RobotContainer.joystick.getLeftTriggerAxis() = 0) {
        intake.IntakeMotor.set(0);
      }

      //if (!RobotContainer.manipButton6.get() && !RobotContainer.manipTrigger.get()){
      if (RobotContainer.joystick.getRightTriggerAxis() = 0) {
        intake.Feeder.set(0);
      }
    //</Shut Off>
  }

    public void runCommand(boolean power) {
        if (power) {
            intake.IntakeMotor.set(.9);
        } else {
            intake.IntakeMotor.set(0);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished; {
        return false;
    }
}}