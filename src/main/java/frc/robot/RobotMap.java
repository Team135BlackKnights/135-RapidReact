package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

public interface RobotMap {

    public interface Drive {

     public final static int 
        FL_ID = 0,
        FR_ID = 1,
        BL_ID = 2,
        BR_ID = 3;

        public SerialPort.Port navXPort = SerialPort.Port.kUSB;

    }

    public interface KOI {
    
     public static final int
        HORIZONTAL_AXIS = 0,
        VERTICAL_AXIS = 1,
        ROTATIONAL_AXIS = 2,
        SLIDER_AXIS = 3;

     public static final int
        LEFT_JOYSTICK = 1, 
        RIGHT_JOYSTICK = 2,
        MANIP_JOYSTICK = 0;
    }
}