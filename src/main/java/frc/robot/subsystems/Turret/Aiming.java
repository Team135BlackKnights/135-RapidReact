package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Aiming extends SubsystemBase {

    public CANSparkMax angleMotor = new CANSparkMax(RobotMap.Turret.R_ID, MotorType.kBrushless);


    public CANSparkMax hoodMotor = new CANSparkMax(RobotMap.Turret.HA_ID, MotorType.kBrushless);
  
    public Encoder shooter, turretAngle, hoodHight; 
  
    public DigitalInput LimitSwitch0, LimitSwitch1;
  

public Aiming() {
    angleMotor.enableVoltageCompensation(12);
    turretAngle = new Encoder(4, 5, false, Encoder.EncodingType.k4X); //rotation
    shooter =     new Encoder(2, 3, false, Encoder.EncodingType.k4X);
    hoodHight =   new Encoder(0, 1, false, Encoder.EncodingType.k4X);

    LimitSwitch0 = new DigitalInput(8);
    LimitSwitch1 = new DigitalInput(9);
}

@Override
public void periodic() {
  // This method will be called once per scheduler run
  SmartDashboard.putNumber("TurretAngleRaw", turretAngle.get());
  SmartDashboard.putBoolean("Limit0", LimitSwitch0.get());
  SmartDashboard.putBoolean("Limit1", LimitSwitch1.get());
}

public void resetEncoders() {
  turretAngle.reset();
  shooter.reset();
  hoodHight.reset();
}
}
