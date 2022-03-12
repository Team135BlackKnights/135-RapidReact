// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hanging;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Hanging.Auto.AutoSolenoids;
//import frc.robot.commands.Hanging.Auto.SelfHang;
import frc.robot.commands.Hanging.Auto.hangLeft;
import frc.robot.commands.Hanging.Auto.hangLeft2;
import frc.robot.commands.Hanging.Auto.hangRight;
import frc.robot.commands.Hanging.Auto.hangRight2;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Hanging.Hang;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoHanging extends SequentialCommandGroup {
  /** Creates a new autoHanging. */
  public autoHanging(Drive drive, Hang hang) {
    addCommands(
      //new SelfHang(drive, hang),
      new ParallelCommandGroup(new hangLeft(hang), new hangRight(hang)),
      new AutoSolenoids(hang),
      new ParallelCommandGroup(new hangLeft2(hang), new hangRight2(hang))
    );
  }
}
