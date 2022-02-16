// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.angleDrive;
import frc.robot.commands.Drive.encoderDrive;
import frc.robot.commands.Drive.resetEncoders;
import frc.robot.commands.Intake.deployIntake;
import frc.robot.commands.Intake.runIntake;
import frc.robot.commands.Intake.Auto.Autofeeder;
import frc.robot.commands.Intake.Auto.Autointake;
import frc.robot.commands.Turret.aimTurret;
import frc.robot.commands.Turret.runShooter;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Turret.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new RedThreeBallAuto. */
  public RedThreeBallAuto(Drive drive, Intake intake, Turret turret) {
      
    super(
          sequence(

              new deployIntake(intake), //intake goes down(Solenoids)


              new resetEncoders(drive), //run encoders, drive to first ball, intake ball
              new ParallelCommandGroup(new encoderDrive(drive, 53.5), new Autointake(intake, 5)), 

              new resetEncoders(drive),
              new angleDrive(drive, 180), //turn robot around to face hub

                                      
              new runShooter(turret),     //than shoot first and second ball
              new Autofeeder(intake, 5),

              new resetEncoders(drive),
              new angleDrive(drive, -160), //turn back around       


              new resetEncoders(drive), //drive forward to next ball(terminal)
              new ParallelCommandGroup(new encoderDrive(drive, 53.5), new Autointake(intake, 5)), //pick up ball

              new resetEncoders(drive),
              new angleDrive(drive, 160), //turn around 

                            
              new runShooter(turret),    //SHOOT!
              new Autofeeder(intake, 5)


          

          ));

  }
}