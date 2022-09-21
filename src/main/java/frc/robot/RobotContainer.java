// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotMap.KOI;
import frc.robot.commands.Drive.tankDrive;
import frc.robot.commands.Drive.Auto.BlueThreeBallAuto;
import frc.robot.commands.Drive.Auto.RedThreeBallAuto;
import frc.robot.commands.Drive.Auto.SimpleAuto;
import frc.robot.commands.Hanging.Hanging;
import frc.robot.commands.Hanging.autoHanging;
import frc.robot.commands.Turret.aimTurret;
import frc.robot.commands.Turret.angleHood;
import frc.robot.commands.Turret.runShooterDistance;

import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Hanging.Hang;
import frc.robot.subsystems.Turret.Turret;

import frc.robot.commands.Intake.deployIntake;
import frc.robot.commands.Intake.runIntake;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxController joystick =  new XboxController(0);
  public static Joystick leftJoystick = new Joystick(RobotMap.KOI.LEFT_JOYSTICK);
  public static Joystick rightJoystick = new Joystick(RobotMap.KOI.RIGHT_JOYSTICK);
  public static Joystick manipJoystick = new Joystick(RobotMap.KOI.MANIP_JOYSTICK);

  public static JoystickButton
  rightTrigger = new JoystickButton(rightJoystick, KOI.TRIGGER_BUTTON),
  rightThumb = new JoystickButton(rightJoystick, KOI.THUMB_BUTTON),
  rightButton3 = new JoystickButton(rightJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
  rightButton5 = new JoystickButton(rightJoystick, KOI.HANDLE_TOP_LEFT_BUTTON),
  rightButton6 = new JoystickButton(rightJoystick, KOI.HANDLE_TOP_RIGHT_BUTTON),
  rightButton10 = new JoystickButton(rightJoystick, KOI.BASE_MIDDLE_RIGHT_BUTTON),
  rightButton11 = new JoystickButton(rightJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
  rightButton12 = new JoystickButton(rightJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON),
  leftTrigger = new JoystickButton(leftJoystick, KOI.TRIGGER_BUTTON),
  leftThumb = new JoystickButton(leftJoystick, KOI.THUMB_BUTTON),

  leftButton3 = new JoystickButton(leftJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
  leftButton4 = new JoystickButton(leftJoystick, KOI.HANDLE_BOTTOM_RIGHT_BUTTON),
  leftButton7 = new JoystickButton(leftJoystick, KOI.BASE_TOP_LEFT_BUTTON),
  leftButton8 = new JoystickButton(leftJoystick, KOI.BASE_TOP_RIGHT_BUTTON),
  leftButton9 = new JoystickButton(leftJoystick, KOI.BASE_MIDDLE_LEFT_BUTTON),
  leftButton10 = new JoystickButton(leftJoystick, KOI.BASE_MIDDLE_RIGHT_BUTTON),
  leftButton11 = new JoystickButton(leftJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
  leftButton12 = new JoystickButton(leftJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON),

  manipTrigger = new JoystickButton(manipJoystick, KOI.TRIGGER_BUTTON),
  manipThumb = new JoystickButton(manipJoystick, KOI.THUMB_BUTTON),
  manipButton3 = new JoystickButton(manipJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
  manipButton4 = new JoystickButton(manipJoystick, KOI.HANDLE_BOTTOM_RIGHT_BUTTON),
  manipButton5 = new JoystickButton(manipJoystick, KOI.HANDLE_TOP_LEFT_BUTTON),
  manipButton7 = new JoystickButton(manipJoystick, KOI.BASE_TOP_LEFT_BUTTON),
  manipButton8 = new JoystickButton(manipJoystick, KOI.BASE_TOP_RIGHT_BUTTON),
  manipButton9 = new JoystickButton(manipJoystick, KOI.BASE_MIDDLE_LEFT_BUTTON),
  manipButton10 = new JoystickButton(manipJoystick, KOI.BASE_MIDDLE_RIGHT_BUTTON),
  manipButton11 = new JoystickButton(manipJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
  manipButton12 = new JoystickButton(manipJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON); //declair all the joystick items that might be used
  
  // The robot's subsystems and commands are defined here...
  public static Turret turret = new Turret(); 
  public static Drive drive = new Drive();
  public static Intake intake = new Intake();
  public static Hang hang = new Hang();
/*
  private final Command Blue = new BlueThreeBallAuto(drive, intake, turret);
  private final Command Red = new RedThreeBallAuto(drive, intake, turret);
  private final Command Simple = new SimpleAuto(drive, intake, turret); */


  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

   /* m_chooser.addOption("BlueAuto", Blue);
    m_chooser.addOption("RedAuto", Red);
    m_chooser.addOption("SimpleAuto", Simple); */

    SmartDashboard.putData(m_chooser);

    turret.setDefaultCommand(new runShooterDistance(turret));
    drive.setDefaultCommand(new tankDrive(drive));
    intake.setDefaultCommand(new runIntake(intake));
    hang.setDefaultCommand(new Hanging(hang));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   /* rightButton3.whenPressed(new deployIntake(intake));
    leftTrigger.whenPressed(new runShooter(turret)); */
    manipTrigger.whenPressed(new angleHood(turret)); 
    manipButton8.whenPressed(new autoHanging(drive, hang));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
