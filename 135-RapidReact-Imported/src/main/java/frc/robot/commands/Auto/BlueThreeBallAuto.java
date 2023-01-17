// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.AutoCommands.Autointake;
import frc.robot.commands.Auto.AutoCommands.encoderDrive;
import frc.robot.commands.Drive.resetEncoders;
import frc.robot.commands.Intake.deployIntake;
import frc.robot.subsystems.DriveRobot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueThreeBallAuto extends SequentialCommandGroup {
    /** Creates a new ThreeBallAuto. */
    public BlueThreeBallAuto(DriveRobot drive, Intake intake, Turret turret) {
      
      super(
            sequence(
//start backwards
                new deployIntake(intake), //intake goes down(Solenoids)


                new ParallelCommandGroup(new encoderDrive(drive, 53.5), new Autointake(intake, 2)),


                                        
               // new ParallelCommandGroup(new Autofeeder(intake, 4), new runShooterAuto(turret, 4)), // shoot first and second ball
   


                new resetEncoders(drive), //drive forward to next ball(terminal), intake
                new ParallelCommandGroup(new encoderDrive(drive, 155), new Autointake(intake, 2)),

                new resetEncoders(drive),
                new encoderDrive(drive, -155)
                                 //SHOOT!
               // new ParallelCommandGroup(new runShooterAuto(turret, 3), new Autofeeder(intake, 3))
                
            ));

    }
}