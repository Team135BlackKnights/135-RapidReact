
package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.Drive;

public class SingleSide extends CommandBase {
    int side;
    Drive drive;
    double Desired, lastOutput, kI, kP, porOut, error, iOut, iTop, iBottom, encodervalue;
    public boolean isFinished;
    /** Creates a new EncoderDrive. */
    public SingleSide(Drive subsystem, double _inchesdesired, int side) {
        Desired = _inchesdesired;
        drive = subsystem; //set drive as drivel
        addRequirements(drive);
    }
//int side = 0 will be right
//int side = 1 will be left
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        iTop = Desired * 1.34;
        iBottom = Desired - (Desired * 1.34);
        kP = 0; //change when testing
        kI = 0; //change when testing
        isFinished = false;
        drive.resetEncoders();
        SmartDashboard.putBoolean("encoder drive", true);




    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        encodervalue = (drive.e_BackLeft.getPosition() + drive.e_BackRight.getPosition() + drive.e_FrontRight.getPosition() + drive.e_FrontLeft.getPosition()) / 4;

        encodervalue = encodervalue / 42;

        encodervalue *= Math.PI * 6;

        error = Desired - encodervalue;

        porOut = error * kP;
        iOut = error * kI;

        if (limit(outputs(), .40, -.40) < .07 && limit(outputs(), .40, -.40) > 0) {
            porOut = .07;
        } else if (limit(outputs(), .40, -.40) > -.07 && limit(outputs(), .40, -.40) < 0) {
            porOut = -.07;
        }

        if (side==0) {
            drive.tankDrive(outputs(), 0);
        }
        else if (side==1) {
            drive.tankDrive(0,outputs());
        }
        drive.tankDrive(outputs(), outputs());

        if (Math.abs(error) < 1) {
            isFinished = true;
        }
    }

    double outputs() {
        if (porOut > iBottom && porOut < iTop) {
            return limit(porOut + iOut, .4, -.4);
        } else {
            return limit(porOut, .4, -.4);
        }
    }

    public static double limit(double x, double upperLimit, double lowerLimit) {
        return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
            x;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished();
    }
}