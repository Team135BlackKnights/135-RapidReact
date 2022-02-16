package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.Drive;

public class angleDrive extends CommandBase {

    boolean isFinished = false;

    public Drive drive;
    public double porOut, lastOut, iOut, iTop, iBottom, desired, error, kp, kI;


    public angleDrive(Drive subsystem, double _angleDesired) {
        desired = _angleDesired;
        drive = subsystem;
        addRequirements(drive);
    }

    public void initialize() {
        kp = 0; //change when testing
        kI = 0; //change when testing
        iTop = desired * 1.34;
        iBottom = desired - (desired * 1.34);
    }

    public void execute() {
        error = drive.navx.getYaw() - desired;
        if (error < -180)
            error += 360;
        else if (error > 180)
            error -= 360;

        error = error / 90;

        porOut = error * kp;
        iOut = error * kI;
        
        if (limit(outputs(), .40, -.40) < .07 && limit(outputs(), .40, -.40) > 0) {
            porOut = .07;
        } else if (limit(outputs(), .40, -.40) > -.07 && limit(outputs(), .40, -.40) < 0) {
            porOut = -.07;
        }
        drive.tankDrive(outputs(), -outputs());
        
        if (Math.abs(error) < 1) {
            isFinished = true;
          }
    }

    double outputs() {
        if (porOut > iBottom && porOut < iTop) {
            return porOut + iOut;
        } else {
            return porOut;
        }
    }

    public static double limit(double x, double upperLimit, double lowerLimit) {
        return x > upperLimit ? upperLimit : x < lowerLimit ? lowerLimit :
            x;
    }


    @Override
    public boolean isFinished() {
      return isFinished;
    }

}