package frc.robot.commands.Hanging;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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
    
    if (RobotContainer.Button9.get()) {
        hang.VerticalHang(1);
    }
    else {
        hang.VerticalHang(0);

    }
  
    if(RobotContainer.Button10.get()) {
        hang.Servo(1);
    }

    else{ 
        hang.Servo(0);
    }
    
} }
