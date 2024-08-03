package frc.robot;

import frc.robot.generated.TunerConstants;

public class Constants {
    public static final double LOOP_INTERVAL = 0.1;

    public enum SYSID {
        Teleop,
        SteerSysId, 
        DriveSysId
    }
    
    public static final Constants.SYSID SYSID_MODE = SYSID.Teleop;

    public static final double RED_AMP_POS_ROTATION= -120;
    public static final double BLUE_AMP_POS_ROTATION= -60;

    public static final double ROTATION_TOLERANCE  = 2;

    public static class Ports{
        public static final int TOP_LAUNCHER = 6;
        public static final int BOTTOM_LAUNCHER = 53;
        public static final int DRIVER_CONTROLLER = 0;
        public static final int CANDLE = 0;
    }

    public static class Launcher{
        public static final int TOP_LAUNCHER_MAX_CURRENT = 120;
        public static final int BOTTOM_LAUNCHER_MAX_CURRENT = 120;
        public static final double TOP_LAUNCHER_VOLTS = 13;
        public static final double BOTTOM_LAUNCHER_VOLTS = 13;
        public static final double INTAKE_VOLTS = 4.5;
        public static final double AMP_VOLTS_TOP = 3;
        public static final double AMP_VOLTS_BOTTOM = 2;
        public static final double LAUNCH_GAP = 1.5;
        public static final double INTAKE_TIME = 2.0;
    }

    public static class Drivetrain{
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps;
        public static final double MAX_ANGULAR_RATE = 1.5 * Math.PI;
    }
}
