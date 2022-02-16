package frc.robot.commands.Hanging;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hang;

public class Hanging extends CommandBase {
    private final frc.robot.subsystems.Hang hang;
    public Hanging(Hang subsystem) {
        hang = subsystem;
        addRequirements(subsystem);
    }
    double x = 0;

    public void initialize() {

    }
    public void execute() {

        Hang.Vert1.set((RobotContainer.hangStick.getRawAxis(1))/2);
        Hang.Vert2.set((RobotContainer.hangStick.getRawAxis(1))/2);
    //time based limiter code so the hanging system doesn't break itself. added this because the substeam 
    //REFUSED to add limit switches for whatever reason
    //i pray this thing is going to be only temporary 
        while((RobotContainer.hangStick.getRawAxis(1))>=.1){
          x = x+(1)*(RobotContainer.hangStick.getRawAxis(1));
        }
        while((RobotContainer.hangStick.getRawAxis(1))<=-0.1){
          x = x-(1)*(RobotContainer.hangStick.getRawAxis(1));
        }
        if (x== 3300) {
         Hang.Vert1.set(0);
         Hang.Vert2.set(0);
        }
        else if ((x==3300)&& (RobotContainer.hangStick.getRawAxis(1)<=-0.1)) {
            Hang.Vert1.set((RobotContainer.hangStick.getRawAxis(1)/2));
            Hang.Vert2.set((RobotContainer.hangStick.getRawAxis(1)/2));
            x = x-(1)*(RobotContainer.hangStick.getRawAxis(1));
          }
        if (x== -0.5) {
          Hang.Vert1.set(0);
          Hang.Vert2.set(0);
        }
        else if ((x==-0.5)&& (RobotContainer.hangStick.getRawAxis(1)>=.1)) {
            Hang.Vert1.set((RobotContainer.hangStick.getRawAxis(1)/2));
            Hang.Vert2.set((RobotContainer.hangStick.getRawAxis(1)/2));
            x = x+(1)*(RobotContainer.hangStick.getRawAxis(1));
          }

    

        if (RobotContainer.rightButton10.get()) {
            hang.Solenoid1.set(true);
            hang.Solenoid2.set(true);
        }
         else if (RobotContainer.rightButton11.get()) {
            hang.Solenoid1.set(false);
            hang.Solenoid2.set(false);
        }
    }



    public double deadband(double Joystick) {
        return Math.abs(Joystick) > .2 ? 0 : Joystick;
              } }