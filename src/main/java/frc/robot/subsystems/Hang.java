package frc.robot.subsystems;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

    
public class Hang extends SubsystemBase{
    public CANSparkMax Vert1;
    public CANSparkMax Vert2;
    public Servo servo1;
    public Servo servo2;
  //  public RelativeEncoder EnV, Env2;
   
    
    

    public Hang(){
        CANSparkMax Vert1 = new CANSparkMax(RobotMap.Hang.V1_ID, MotorType.kBrushless);
        CANSparkMax Vert2 = new CANSparkMax(RobotMap.Hang.V2_ID, MotorType.kBrushless);
        Servo servo1 = new Servo(RobotMap.Hang.S1_ID);
        Servo servo2 = new Servo(RobotMap.Hang.S2_ID);
    //    EnV = Vert1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
      //  Env2 = Vert2.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

      
        //put all motor setup in here
        try {

        }
        finally{
            Vert1.close();
            Vert2.close();
            servo1.close();
            servo2.close();
     
        }
    }
    public void VerticalHang(double power){
        Vert1.set(power);
        Vert2.set(power);
    }

    public void Servo(double power) {
        servo1.set(power);
        servo2.set(power);
    }
    
}

