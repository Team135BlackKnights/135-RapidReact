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

    private void addRequirements(Hang subsystem) {}
   
    public void initialize() {

    }
    public void execute() {

        hang.VerticalHang(deadband(RobotContainer.manipJoystick.getRawAxis(RobotMap.KOI.SLIDER_AXIS)));

        if (!hang.Limit1.get() || !hang.Limit2.get()){
            hang.Servo(0);
        }
        else if (RobotContainer.Button10.get()) {
            hang.Servo(.8);
        } else if (RobotContainer.Button9.get()) {
            hang.Servo(-.8);
        } else {
            hang.Servo(0);
        }

    }



    public double deadband(double Joystick) {
        return Joystick < .2 && Joystick > 0 ? 0 : Joystick > -.2 && Joystick < 0 ? 0 : Joystick;
    }
}