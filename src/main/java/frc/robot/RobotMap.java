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
    public interface Hang {
        public final static int
            V1_ID = 4,
            V2_ID = 5,
            H1_ID = 6,
            H2_ID = 7;
    }
    public interface Intake { 

     public final static int 
        LI_ID = 0,
        RI_ID = 1,
        S1_ID = 0,
        S2_ID = 1,
        S3_ID = 2,
        S4_ID = 3;
    }

    public interface Turret{

     public final static int
        PL_ID = 20,
        PR_ID = 21;

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
        
     public static final int
        TRIGGER = 1,
        HANDLE_BUTTON1 = 3,
        HANDLE_BUTTON2 = 4;
    }
}
