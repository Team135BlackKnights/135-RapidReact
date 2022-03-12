package frc.robot.commands.Hanging.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanging.Hang;

public class hangRight extends CommandBase{
    private final Hang hang;
    boolean isFinished;

    public hangRight(Hang subsystem) {
        hang = subsystem;
        addRequirements(subsystem);
    }
    double x = 0;
    boolean wah = false;
    public void initialize() {

    }
    public void execute() {
        SmartDashboard.putNumber("Hang X", x);
        if (x==110) {
            hang.Vert2.set(0);
             wah = true;
        }
        else if (x <=- 5) {
            hang.Vert2.set(0);
             wah = false;
             isFinished = true;
        }
        else if (x<110 && wah==false) {
            hang.Vert2.set(1);
            x= x+1;
        }
        if (wah==true) {
            x= x-1;
            hang.Vert2.set(-1);
        }
        

}
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
