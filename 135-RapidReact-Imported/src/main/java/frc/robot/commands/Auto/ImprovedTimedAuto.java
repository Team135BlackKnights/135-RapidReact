// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.AutoCommands.Autofeeder;
import frc.robot.commands.Auto.AutoCommands.Autointake;
import frc.robot.commands.Auto.AutoCommands.RunShooterAuto;
import frc.robot.commands.Auto.AutoCommands.TimeDrive;
import frc.robot.commands.Hanging.deployHang;
import frc.robot.commands.Intake.deployIntake;
import frc.robot.subsystems.DriveRobot;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ImprovedTimedAuto extends SequentialCommandGroup {
  /** Creates a new ImprovedTimedAuto. */
  public ImprovedTimedAuto(Intake intake, DriveRobot drive, Turret turret, Hang hang) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new deployIntake(intake),
      new deployHang(hang),
      new ParallelCommandGroup(new TimeDrive(drive, 2), new Autointake(intake, 5)),
      new ParallelCommandGroup(new RunShooterAuto(turret), new Autofeeder(intake, 10))
    );
  }
}
