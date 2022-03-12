package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.Intake;

public class Autofeeder extends CommandBase {
    /** Creates a new Autofeeder. */
    double time;
    Timer timer;
    Intake intake;
    boolean isFinished = false;

    public Autofeeder(Intake subsystem, double m_time) 
    {

        time = m_time;
        intake = subsystem;
        addRequirements(subsystem);

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        timer.start();
     
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(timer.get()>time) 
        {
            intake.Feeder.set(0);
            isFinished = true;
        }

        if(timer.get()> 2) 
        {
            intake.Feeder.set(.5);
        }

    
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
     {
        timer.stop();
        timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
     {
        return isFinished;
    }
}