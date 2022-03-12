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

     /*   if (RobotContainer.rightButton11.get()) {
            hang.Solenoid2.set(Value.kForward);
            hang.Solenoid1.set(Value.kForward);
        } else if (RobotContainer.rightButton12.get()) {
            hang.Solenoid2.set(Value.kOff);
            hang.Solenoid1.set(Value.kOff);
        } else {
            hang.Solenoid2.set(Value.kOff);
            hang.Solenoid1.set(Value.kOff);
        } */

    }

    public double deadband(double Joystick) {
        return Math.abs(Joystick) < .2 ? 0 : Joystick;
    }
} 