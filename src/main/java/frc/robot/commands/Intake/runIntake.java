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

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (RobotContainer.rightTrigger.get()) {
            //Take in
            intake.IntakeMotor.set(.9);
        } else if (RobotContainer.rightButton6.get()) {
            //Spit out
            intake.IntakeMotor.set(-.9);
        } else {
            //Stop draining the battery
            intake.IntakeMotor.set(0);
        }

        if (RobotContainer.rightThumb.get()) {
            //pull in feeder
            intake.Feeder.set(.8);
        } else if (RobotContainer.rightButton10.get()) {
            //move down feeder
            intake.Feeder.set(-.8);
        } else {
            //don't move
            intake.Feeder.set(0);
        }
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
    public boolean isFinished() {
        return false;
    }
}