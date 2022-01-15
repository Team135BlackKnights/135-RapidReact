package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

    
public class Hang extends SubsystemBase{
    public CANSparkMax Vert1;
    public CANSparkMax Vert2;
    public CANSparkMax Hor1;
    public CANSparkMax Hor2;

    public Hang(){
        CANSparkMax Vert1 = new CANSparkMax(RobotMap.Hang.V1_ID, MotorType.kBrushless);
        CANSparkMax Vert2 = new CANSparkMax(RobotMap.Hang.V2_ID, MotorType.kBrushless);
        CANSparkMax Hor1 = new CANSparkMax(RobotMap.Hang.H1_ID, MotorType.kBrushless);
        CANSparkMax Hor2 = new CANSparkMax(RobotMap.Hang.H2_ID, MotorType.kBrushless);

        Vert1.close();
        Vert2.close();
        Hor1.close();
        Hor2.close();
    }
    public void VerticalHang(double power){
        Vert1.set(power);
        Vert2.set(power);
    }
    public void HorHang(double power){
        Hor1.set(power);
        Hor2.set(power);
    
}
}

