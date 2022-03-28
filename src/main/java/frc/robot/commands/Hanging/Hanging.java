package frc.robot.commands.Hanging;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hang;

public class Hanging extends CommandBase {
    private final Hang hang;
    public Hanging(Hang subsystem) {
        hang = subsystem;
        addRequirements(subsystem);
    }

    public void execute() {        
        hang.Vert1.set(deadband(-RobotContainer.manipJoystick.getRawAxis(1)/1.5));
        hang.Vert2.set(deadband(RobotContainer.manipJoystick.getRawAxis(1)/1.5));

        SmartDashboard.putNumber("Hang 1", hang.Vert1.getOutputCurrent());
        SmartDashboard.putNumber("Hang 2", hang.Vert2.getOutputCurrent());
    }
    public double deadband(double Joystick) {
        return Math.abs(Joystick) < .2 ? 0 : Joystick;
    }
} 