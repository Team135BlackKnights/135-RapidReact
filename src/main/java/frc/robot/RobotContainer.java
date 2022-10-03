// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap.KOI;
import frc.robot.commands.Auto.BlueThreeBallAuto;
import frc.robot.commands.Auto.ImprovedTimedAuto;
import frc.robot.commands.Auto.RedThreeBallAuto;
import frc.robot.commands.Auto.AutoCommands.TimeDrive;
import frc.robot.commands.Auto.AutoCommands.encoderDrive;
import frc.robot.commands.Drive.tankDrive;
import frc.robot.commands.Hanging.Hanging;
import frc.robot.commands.Turret.ImprovedAiming;
import frc.robot.commands.Turret.aimTurret;
import frc.robot.commands.Turret.angleHood;
import frc.robot.commands.Turret.runShooterDistance;
import frc.robot.commands.Turret.runShooterHoodDistance;
import frc.robot.subsystems.DriveRobot;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret.Aiming;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.commands.Intake.deployIntake;
import frc.robot.commands.Intake.runIntake;
import frc.robot.commands.Hanging.deployHang;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxController controller1 = new XboxController(0);
  public static XboxController manipController = new XboxController(1);
  public static Joystick leftJoystick = new Joystick(RobotMap.KOI.LEFT_JOYSTICK);
  public static Joystick rightJoystick = new Joystick(RobotMap.KOI.RIGHT_JOYSTICK);
  public static Joystick manipJoystick = new Joystick(RobotMap.KOI.MANIP_JOYSTICK);

  public static JoystickButton rightTrigger = new JoystickButton(rightJoystick, KOI.TRIGGER_BUTTON),
      rightThumb = new JoystickButton(rightJoystick, KOI.THUMB_BUTTON),
      rightButton3 = new JoystickButton(rightJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
      rightButton4 = new JoystickButton(rightJoystick, KOI.HANDLE_BOTTOM_RIGHT_BUTTON),
      rightButton5 = new JoystickButton(rightJoystick, KOI.HANDLE_TOP_LEFT_BUTTON),
      rightButton6 = new JoystickButton(rightJoystick, KOI.HANDLE_TOP_RIGHT_BUTTON),
      rightButton10 = new JoystickButton(rightJoystick, KOI.BASE_MIDDLE_RIGHT_BUTTON),
      rightButton11 = new JoystickButton(rightJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
      rightButton12 = new JoystickButton(rightJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON),
      leftTrigger = new JoystickButton(leftJoystick, KOI.TRIGGER_BUTTON),
      leftThumb = new JoystickButton(leftJoystick, KOI.THUMB_BUTTON),

      leftButton3 = new JoystickButton(leftJoystick, KOI.HANDLE_BOTTOM_LEFT_BUTTON),
      leftButton4 = new JoystickButton(leftJoystick, KOI.HANDLE_BOTTOM_RIGHT_BUTTON),
      leftButton5 = new JoystickButton(leftJoystick, KOI.HANDLE_TOP_LEFT_BUTTON),
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
      manipButton6 = new JoystickButton(manipJoystick, KOI.HANDLE_TOP_RIGHT_BUTTON),
      manipButton7 = new JoystickButton(manipJoystick, KOI.BASE_TOP_LEFT_BUTTON),
      manipButton8 = new JoystickButton(manipJoystick, KOI.BASE_TOP_RIGHT_BUTTON),
      manipButton9 = new JoystickButton(manipJoystick, KOI.BASE_MIDDLE_LEFT_BUTTON),
      manipButton10 = new JoystickButton(manipJoystick, KOI.BASE_MIDDLE_RIGHT_BUTTON),
      manipButton11 = new JoystickButton(manipJoystick, KOI.BASE_BOTTOM_LEFT_BUTTON),
      manipButton12 = new JoystickButton(manipJoystick, KOI.BASE_BOTTOM_RIGHT_BUTTON);
  // declair all the joystick items that might be used

  // The robot's subsystems and commands are defined here...
  public static Turret turret = new Turret();
  public static Aiming aiming = new Aiming();
  public static DriveRobot drive = new DriveRobot();
  public static Intake intake = new Intake();
  public static Hang hang = new Hang();

  private final Command Blue = new BlueThreeBallAuto(drive, intake, turret);
  private final Command Red = new RedThreeBallAuto(drive, intake, turret);
  private final Command TimedAuto = new frc.robot.commands.Auto.TimedAuto(drive, turret, intake);
  private final Command ImprovedTimedAuto = new ImprovedTimedAuto(intake, drive, turret, hang);
  private final Command Taxi = new TimeDrive(drive, 1.2);
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_chooser.addOption("BlueAuto", Blue);
    m_chooser.addOption("RedAuto", Red);
    m_chooser.addOption("Taxi", Taxi);
    m_chooser.setDefaultOption("ImprovedTimedAuto", ImprovedTimedAuto);
    m_chooser.addOption("TimedAuto", TimedAuto);

    SmartDashboard.putData(m_chooser);

    aiming.setDefaultCommand(new ImprovedAiming(aiming));
    turret.setDefaultCommand(new runShooterDistance(turret));
    
    drive.setDefaultCommand(new tankDrive(drive));
    intake.setDefaultCommand(new runIntake(intake));
    hang.setDefaultCommand(new Hanging(hang));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    manipThumb.whenPressed(new deployIntake(intake));
    leftThumb.whenPressed(new angleHood(turret));
    manipButton7.whenPressed(new deployHang(hang));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
