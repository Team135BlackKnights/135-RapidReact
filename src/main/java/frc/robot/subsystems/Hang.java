package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

    
public class Hang extends SubsystemBase{
    public CANSparkMax Vert1;
    public CANSparkMax Vert2;
    public Solenoid Solenoid1, Solenoid2;
    public DigitalInput Limit1;
    public DigitalInput Limit2;
    public DigitalInput Limit3;
     public  DigitalInput Limit4;
     public DigitalInput ColorSensor;

  //  public RelativeEncoder EnV, Env2;
   
    
    

    public Hang(){
        CANSparkMax Vert1 = new CANSparkMax(RobotMap.Hang.V1_ID, MotorType.kBrushless);
        CANSparkMax Vert2 = new CANSparkMax(RobotMap.Hang.V2_ID, MotorType.kBrushless);
        Solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.Hang.So1_ID);
        Solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.Hang.So2_ID);
        Limit1 = new DigitalInput(RobotMap.Hang.Li_ID);
        Limit2 = new DigitalInput(RobotMap.Hang.Li2_ID);
        Limit3 = new DigitalInput(RobotMap.Hang.Li3_ID);
        Limit4 = new DigitalInput(RobotMap.Hang.Li4_ID);
      
        //put all motor setup in here
        try {

        }
        finally{
            Vert1.close();
            Vert2.close();
            Solenoid1.close();
            Solenoid2.close();
           
     
        }
    }
    public void VerticalHang(double power){
        Vert1.set(power);
        Vert2.set(power);
    }

   


    
}

