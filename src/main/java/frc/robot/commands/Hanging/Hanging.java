package frc.robot.commands.Hanging;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hanging.Hang;

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
    if (x<= -5) {
        x= -5;
    }
    else if (x>=70) {
        x=70;
    }
        else if ((RobotContainer.manipJoystick.getRawAxis(1))>=.1){
          x = x-(RobotContainer.manipJoystick.getRawAxis(1));
        }
        else if ((RobotContainer.manipJoystick.getRawAxis(1))<=-0.1){
          x = x-(RobotContainer.manipJoystick.getRawAxis(1));
        }
        
        SmartDashboard.putNumber("Hang X", x);
       
        
        
        if ((x==70)&& (RobotContainer.manipJoystick.getRawAxis(1)>=0.1)) {
            hang.Vert1.set((RobotContainer.manipJoystick.getRawAxis(1)/2));
            hang.Vert2.set((RobotContainer.manipJoystick.getRawAxis(1)/2));
            x = x-(1)*(RobotContainer.manipJoystick.getRawAxis(1));
          }
        
        else if ((x==-5)&& (RobotContainer.manipJoystick.getRawAxis(1)<=-.1)) {
            hang.Vert1.set((RobotContainer.manipJoystick.getRawAxis(1)/2));
            hang.Vert2.set((RobotContainer.manipJoystick.getRawAxis(1)/2));
            x = x-(1)*(RobotContainer.manipJoystick.getRawAxis(1));
          }
          else if (x>= 70) { //change x==abc with testing
            hang.Vert1.set(0);
            hang.Vert2.set(0);
            x = 70;
           }
        else if ( x <= -5) {
            hang.Vert1.set(0);
            hang.Vert2.set(0);
            x = -5;
          }
        else {
            hang.VerticalHang((RobotContainer.manipJoystick.getRawAxis(1)/2));
        }


        if (RobotContainer.rightButton11.get()) {
            hang.Solenoid2.set(Value.kForward);

        } else if (RobotContainer.rightButton12.get()) {
            hang.Solenoid2.set(Value.kOff);
        } else {
            hang.Solenoid2.set(Value.kOff);

        }

        if (RobotContainer.leftButton11.get()) {
            hang.Solenoid3.set(Value.kForward);

        } else if (RobotContainer.leftButton12.get()) {
            hang.Solenoid3.set(Value.kOff);
        } else {
            hang.Solenoid3.set(Value.kOff);
        }

    }

    public double deadband(double Joystick) {
        return Math.abs(Joystick) > .2 ? 0 : Joystick;
    }
}