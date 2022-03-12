package frc.robot.commands.Hanging.Auto;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanging.Hang;

public class AutoSolenoids extends CommandBase{
    private final Hang hang;
    boolean isFinished = false;

    public AutoSolenoids(Hang subsystem) {
        hang = subsystem;
        addRequirements(subsystem);
    }

    public void initialize() {

    }
    public void execute() {
        hang.Solenoid2.set(Value.kForward);
        hang.Solenoid3.set(Value.kForward);
        isFinished = true;
}
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
