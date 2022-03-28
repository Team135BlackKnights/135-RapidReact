package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Aiming extends SubsystemBase {

  public CANSparkMax angleMotor = new CANSparkMax(RobotMap.Turret.R_ID, MotorType.kBrushless);
  public RelativeEncoder turretAngle;
  public AnalogInput LimitSwitch;

  public double EndPos;
  public Boolean Thresholding;

  public Aiming() {
    angleMotor.enableVoltageCompensation(12);
    angleMotor.setSmartCurrentLimit(6);
    angleMotor.setInverted(true);
    angleMotor.burnFlash();

    turretAngle = angleMotor.getEncoder();
    LimitSwitch = new AnalogInput(0);

  }

  public boolean LimitValue(AnalogInput LimitSwitch) {
    return LimitSwitch.getVoltage() >= .1 ? true : false;
  }

  @Override
  public void periodic() {

    if (LimitValue(LimitSwitch) && Thresholding) {
      angleMotor.set(-.4);
    } else {
      Thresholding = false;
    }

    // <Light Controll>
    if (-RobotContainer.manipJoystick.getRawAxis(3) > 0) {
      NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("ledMode").setNumber(3);
    } else if (-RobotContainer.manipJoystick.getRawAxis(3) < 0) {
      NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("ledMode").setNumber(1);
    }
    // </Light Controll>

    SmartDashboard.putNumber("TurretAngleRaw", turretAngle.getPosition());
    SmartDashboard.putBoolean("Limit0", LimitValue(LimitSwitch));
  }

  public void resetEncoders() {
    turretAngle.setPosition(0);
  }
}
