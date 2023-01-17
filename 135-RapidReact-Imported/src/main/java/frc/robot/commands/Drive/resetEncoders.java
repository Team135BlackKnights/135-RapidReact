package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveRobot;

public class resetEncoders extends CommandBase {
  DriveRobot drive;

  public resetEncoders(DriveRobot subsystem) {
    drive = subsystem;
  }

  @Override
  public void initialize() {
    // Runs method to reset the encoders from drivetrain
    //drive.navx.reset();
    addRequirements(drive);
  }
}
