package frc.robot.commands.Hanging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanging.Hang;

public class hangLeft extends CommandBase{
    private final Hang hang;
    

    public hangLeft(Hang subsystem) {
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
            Hang.Vert1.set(0);
            wah = true;
            
        }
        
        else if (x==-5) {
            Hang.Vert1.set(0);
            wah = false;
        }
        else if (x<110) {
            Hang.Vert1.set(1);
            x= x+1;
        }
    
        if (wah==true) {
            x= x-1;
            Hang.vert1.set(-1);
        }

}

}
