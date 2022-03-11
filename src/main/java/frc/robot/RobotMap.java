
package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C.Port;

public interface RobotMap {

    public interface Drive {
     public final static int 
        FL_ID = 11,
        FR_ID = 42,
        BL_ID = 12,
        BR_ID = 41;

        public final SerialPort.Port navXPort = SerialPort.Port.kUSB;

    }
    public interface Hang {
        public final static int
            V1_ID = 51,
            V2_ID = 52,
            So3_ID = 10, So4_ID = 11,
            So5_ID = 12, So6_ID = 13,
            Li_ID = 0,
            Li2_ID = 1,
            Li3_ID = 4,
            Li4_ID = 5,
            Co_S_ID = 6;
    }
    public interface Intake { 
     public final static int 
        InM_ID = 21,
        FM_ID = 22,
        So1_ID = 14, So2_ID = 15;
        public final I2C.Port colorPort = I2C.Port.kOnboard;
    }

    public interface Turret{
     public final static int
        PL_ID = 34,
        PR_ID = 23,
        R_ID = 35, //rotate ID
        HA_ID = 32; //Hood Angle
    }   

    public interface KOI {
     public static final int
        HORIZONTAL_AXIS = 0,
        VERTICAL_AXIS = 1,
        ROTATIONAL_AXIS = 2,
        SLIDER_AXIS = 5;

     public static final int
        LEFT_JOYSTICK = 0, 
        RIGHT_JOYSTICK = 1,
        MANIP_JOYSTICK = 2;
                
     public static final int
     TRIGGER_BUTTON = 1, THUMB_BUTTON = 2,

     HANDLE_BOTTOM_LEFT_BUTTON = 3, HANDLE_BOTTOM_RIGHT_BUTTON = 4, HANDLE_TOP_LEFT_BUTTON = 5,
     HANDLE_TOP_RIGHT_BUTTON = 6,

     BASE_TOP_LEFT_BUTTON = 7, BASE_TOP_RIGHT_BUTTON = 8, BASE_MIDDLE_LEFT_BUTTON = 9,
     BASE_MIDDLE_RIGHT_BUTTON = 10, BASE_BOTTOM_LEFT_BUTTON = 11, BASE_BOTTOM_RIGHT_BUTTON = 12;
    }
}