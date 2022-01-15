package frc.robot.commands.Hanging;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hang;
public class Hanging extends CommandBase{
    private final frc.robot.subsystems.Hang hang;
    public Hanging(Hang subsystem) {
        hang = subsystem;
        addRequirements(subsystem);

    }

private void addRequirements(Hang subsystem) {
    }
public void initialize(){

}
public void execute() {
    hang.VerticalHang(0);
    hang.HorHang(0);
}





    
}
