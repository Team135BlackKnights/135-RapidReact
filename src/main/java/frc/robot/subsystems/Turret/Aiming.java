package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Aiming extends SubsystemBase {

    public CANSparkMax angleMotor = new CANSparkMax(RobotMap.Turret.R_ID, MotorType.kBrushless);
    public Encoder turretAngle; 
    public AnalogInput LimitSwitch0, LimitSwitch1;
  
    public double EndPos;

public Aiming() {
    angleMotor.enableVoltageCompensation(12);
    turretAngle = new Encoder(4, 5, false, Encoder.EncodingType.k4X); //rotation

    LimitSwitch0 = new AnalogInput(0);
    LimitSwitch1 = new AnalogInput(2);
}

public boolean LimitValue(AnalogInput LimitSwitch){
  return LimitSwitch.getVoltage() >= .1 ? true : false;
}

@Override
public void periodic() {
  // This method will be called once per scheduler run
  SmartDashboard.putNumber("TurretAngleRaw", turretAngle.get());
  SmartDashboard.putBoolean("Limit0", LimitValue(LimitSwitch0));
  SmartDashboard.putBoolean("Limit1", LimitValue(LimitSwitch1));
}

public void resetEncoders() {
  turretAngle.reset();
}
}
