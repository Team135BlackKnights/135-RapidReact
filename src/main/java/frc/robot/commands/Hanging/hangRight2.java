package frc.robot.commands.Hanging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanging.Hang;

public class hangRight2 extends CommandBase{
    private final Hang hang;
    

    public hangRight2(Hang subsystem) {
        hang = subsystem;
        addRequirements(subsystem);
    }
    double x = 0;
    boolean wah = false;
    public void initialize() {

    }
    public void execute() {
        SmartDashboard.putNumber("Hang X", x);
        if (x==220) {
            Hang.Vert2.set(0);
            wah = true;
        }
        else if (x==-5) {
            Hang.Vert2.set(0);
        }
        else if (x<220) {
            Hang.Vert2.set(1);
        }
        if (wah==true) {
            x = x-1;
            Hang.Vert2.set(-1);
        }
        

}

}
