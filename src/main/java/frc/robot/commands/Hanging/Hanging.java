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
    double x = 0;

    public void initialize() {
        hang.LiftSolenoid.set(Value.kReverse);
        hang.LiftSolenoid.set(Value.kOff);
    }
    public void execute() {
    //time based limiter code so the hanging system doesn't break itself. added this because the substeam 
    //REFUSED to add limit switches for whatever reason
    //i pray this thing is going to be only temporary 
  
        
        SmartDashboard.putNumber("Hang X", x);
       
        
        
        if ((x==220)&& (RobotContainer.manipJoystick.getRawAxis(1)>=0.1)) {
            hang.Vert1.set(deadband(RobotContainer.manipJoystick.getRawAxis(1)/2));
            hang.Vert2.set(deadband(RobotContainer.manipJoystick.getRawAxis(1)/2));
            x = x-((1)*(RobotContainer.manipJoystick.getRawAxis(1)));
          }
        
        else if ((x==-5)&& (RobotContainer.manipJoystick.getRawAxis(1)<=-.1)) {
            hang.Vert1.set(deadband(RobotContainer.manipJoystick.getRawAxis(1)/2));
            hang.Vert2.set(deadband(RobotContainer.manipJoystick.getRawAxis(1)/2));
            x = x-((1)*(RobotContainer.manipJoystick.getRawAxis(1)));
          }
          else if (x>= 220) { //change x==abc with testing
            hang.Vert1.set(0);
            hang.Vert2.set(0);
            x = 220;
           }
        else if ( x <= -5) {
            hang.Vert1.set(0);
            hang.Vert2.set(0);
            x = -5;
          }
        else {
            hang.VerticalHang(deadband(RobotContainer.manipJoystick.getRawAxis(1)/2));
        }

        if (RobotContainer.manipButton5.get()) {
            hang.LiftSolenoid.set(Value.kForward);
        } else if (RobotContainer.manipButton6.get()) {
            hang.LiftSolenoid.set(Value.kReverse);
        } else if (RobotContainer.manipButton7.get()){
            hang.LiftSolenoid.set(Value.kOff);
        }

        SmartDashboard.putString("LiftSolenoid", hang.LiftSolenoid.get().toString());

        if (RobotContainer.manipButton11.get()) {
            hang.HookSolenoid.set(Value.kForward);
        } else if (RobotContainer.manipButton12.get()) {
            hang.HookSolenoid.set(Value.kReverse);
        } else if (RobotContainer.leftButton12.get()){
            hang.HookSolenoid.set(Value.kOff);
        }

        SmartDashboard.putString("HookSolenoid", hang.HookSolenoid.get().toString());

    }

    public double deadband(double Joystick) {
        return Math.abs(Joystick) < .2 ? 0 : Joystick;
    }
} 