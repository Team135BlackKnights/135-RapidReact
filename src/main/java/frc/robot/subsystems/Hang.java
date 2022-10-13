// all of this is subject to change and we dont know much of anything about hang right now
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Hang extends SubsystemBase {

    public CANSparkMax Vert1 = new CANSparkMax(RobotMap.Hang.V1_ID, MotorType.kBrushless);
    public CANSparkMax Vert2 = new CANSparkMax(RobotMap.Hang.V2_ID, MotorType.kBrushless);
    public DoubleSolenoid Solenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, RobotMap.Hang.So12_ID, RobotMap.Hang.So13_ID);

    public Hang() {
        Vert1.setIdleMode(IdleMode.kBrake);
        Vert2.setIdleMode(IdleMode.kBrake);
        Vert1.setSmartCurrentLimit(25);
        Vert2.setSmartCurrentLimit(25);

        Vert2.burnFlash();
        Vert1.burnFlash();

        Solenoid.set(Value.kForward);
    }

    public void VerticalHang(double power) {
        Vert1.set(power);
        Vert2.set(-power);
    }

}
