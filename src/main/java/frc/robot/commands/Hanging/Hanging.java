package frc.robot.commands.Hanging;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Hang;

public class Hanging extends CommandBase {
    private final frc.robot.subsystems.Hang hang;
    public Hanging(Hang subsystem) {
        hang = subsystem;
        addRequirements(subsystem);
    }
   
    public void initialize() {

    }
    public void execute() {

        if (!hang.Limit1.get() || !hang.Limit2.get()){
           hang.VerticalHang(0);
        } else {
            hang.VerticalHang(deadband(RobotContainer.manipJoystick.getRawAxis(RobotMap.KOI.SLIDER_AXIS))); 
        }


        if (RobotContainer.Button10.get()) {
            hang.Solenoid1.set(true);
            hang.Solenoid2.set(true);
        }
         else if (RobotContainer.Button9.get()) {
            hang.Solenoid1.set(false);
            hang.Solenoid2.set(false);
        }
    }



    public double deadband(double Joystick) {
        return Joystick < .2 && Joystick > 0 ? 0 : Joystick > -.2 && Joystick < 0 ? 0 : Joystick;
    }
}