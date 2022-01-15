package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.Drive;

public class resetEncoders extends CommandBase {
  Drive drive;

  public resetEncoders(Drive subsystem) {
    drive = subsystem;
  }

  @Override
  public void initialize() {
    // Runs method to reset the encoders from drivetrain
    drive.resetEncoders();
    drive.navx.reset();
    addRequirements(drive);
  }
}
