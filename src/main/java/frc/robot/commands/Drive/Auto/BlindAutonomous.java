
package frc.robot.commands.Drive.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.encoderDrive;
import frc.robot.commands.Drive.resetEncoders;
import frc.robot.subsystems.Drive.Drive;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlindAutonomous extends SequentialCommandGroup {
  /** Creates a new Gline. */
  public BlindAutonomous(Drive drive) {
   super (
     sequence(
     new resetEncoders(drive),
     new encoderDrive(drive, 38.2)
     
     
     ));


  }
}
