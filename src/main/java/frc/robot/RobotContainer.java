// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive.tankDrive;
<<<<<<< Updated upstream
import frc.robot.commands.Turret.runShooter;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Turret.Turret;
=======
import frc.robot.commands.Intake.deployIntake;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
>>>>>>> Stashed changes

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static Joystick leftJoystick = new Joystick(RobotMap.KOI.LEFT_JOYSTICK);
  public static Joystick rightJoystick = new Joystick(RobotMap.KOI.RIGHT_JOYSTICK);
  public static Joystick manipJoystick = new Joystick(RobotMap.KOI.MANIP_JOYSTICK);
  public static JoystickButton rightTrigger = new JoystickButton(rightJoystick, RobotMap.KOI.Trigger_Button);
  public static JoystickButton Button7 = new JoystickButton(leftJoystick, RobotMap.KOI.Base_Top_Left_Button);
  public static JoystickButton Button2 = new JoystickButton(leftJoystick, RobotMap.KOI.THUMB_BUTTON);
  public static JoystickButton Button8 = new JoystickButton(leftJoystick, RobotMap.KOI.Button8);
  public static JoystickButton Button9 = new JoystickButton(leftJoystick, RobotMap.KOI.Button9);
  public static JoystickButton Button10 = new JoystickButton(leftJoystick, RobotMap.KOI.Button10);
  public static JoystickButton Button11 = new JoystickButton(leftJoystick, RobotMap.KOI.Button11);
  public static JoystickButton Rbutton2 = new JoystickButton(rightJoystick, RobotMap.KOI.THUMB_BUTTON) ;
  
  // The robot's subsystems and commands are defined here...
<<<<<<< Updated upstream
  public static Turret turret = new Turret(); 
=======
  public static Drive drive = new Drive();
  public static Intake intake = new Intake();
>>>>>>> Stashed changes

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

<<<<<<< Updated upstream
    turret.setDefaultCommand(new runShooter(turret));
    
=======
    drive.setDefaultCommand(new tankDrive(drive));
    intake.setDefaultCommand(new deployIntake(intake));

>>>>>>> Stashed changes
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
