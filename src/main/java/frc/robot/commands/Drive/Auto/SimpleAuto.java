
package frc.robot.commands.Drive.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.angleDrive;
import frc.robot.commands.Drive.encoderDrive;
import frc.robot.commands.Drive.resetEncoders;
import frc.robot.commands.Intake.deployIntake;
import frc.robot.commands.Intake.runIntake;
import frc.robot.commands.Turret.aimTurret;
import frc.robot.commands.Turret.runShooter;
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


      new resetEncoders(drive), //run encoders, drive to first ball
      new encoderDrive(drive, 53.5),


      new runIntake(intake), //intake second ball(already have one prematch)
      
      
      new resetEncoders(drive),
      new angleDrive(drive, 180), //turn robot around to face hub


      new aimTurret(turret), //calibrate shooter and than shoot first and second ball
      new runShooter(turret)

     
     
     ));


  }
}
