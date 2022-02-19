// all of this is subject to change and we dont know much of anything about hang right now
package frc.robot.subsystems.Hanging;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class Hang extends SubsystemBase {
    public static CANSparkMax Vert1;
    public static CANSparkMax Vert2;
    public DoubleSolenoid Solenoid2, Solenoid3;
    public DigitalInput Limit1;
    public DigitalInput Limit2;
    public DigitalInput Limit3;
    public DigitalInput Limit4;
    public DigitalInput ColorSensor;

    //  public RelativeEncoder EnV, Env2;




    public Hang() {
        CANSparkMax Vert1 = new CANSparkMax(RobotMap.Hang.V1_ID, MotorType.kBrushless);
        CANSparkMax Vert2 = new CANSparkMax(RobotMap.Hang.V2_ID, MotorType.kBrushless);
        Solenoid2 = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, RobotMap.Hang.So3_ID, RobotMap.Hang.So4_ID);
        Solenoid3 = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, RobotMap.Hang.So5_ID, RobotMap.Hang.So6_ID);

    }
    public void VerticalHang(double power) {
        Vert1.set(power);
        Vert2.set(power);
    }

}