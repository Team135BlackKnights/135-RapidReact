
package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.AutoCommands.Autofeeder;
import frc.robot.commands.Auto.AutoCommands.Autointake;
import frc.robot.commands.Auto.AutoCommands.runShooterAuto;
import frc.robot.commands.Drive.angleDrive;
import frc.robot.commands.Drive.encoderDrive;
import frc.robot.commands.Drive.resetEncoders;
import frc.robot.commands.Intake.deployIntake;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Turret.Turret;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAuto extends SequentialCommandGroup {
  /** Creates a new Gline. */
  public SimpleAuto(Drive drive, Intake intake, Turret turret) {


   super (

     sequence(

    
      new deployIntake(intake), //intake goes down(Solenoids)


      new resetEncoders(drive), //run encoders, drive to first ball, intake
      new ParallelCommandGroup(new encoderDrive(drive, 53.5), new Autointake(intake, 5)), 
      
      
      new resetEncoders(drive),
      new angleDrive(drive, 180), //turn robot around to face hub


     
      new ParallelCommandGroup(new runShooterAuto(turret, 5), new Autofeeder(intake, 5))
     
     
     ));


  }
}
