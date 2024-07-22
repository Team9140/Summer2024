package frc.robot;

import frc.robot.generated.TunerConstants;

public class Constants {
    public static class Ports{

        public static final int TOP_LAUNCHER = 1;
        public static final int BOTTOM_LAUNCHER = 2;
        public static final int DRIVER_CONTROLLER = 0;
    }

    public static class Launcher{
        public static final int TOP_LAUNCHER_MAX_CURRENT = 100;
        public static final int BOTTOM_LAUNCHER_MAX_CURRENT = 100;
        public static final int TOP_LAUNCHER_VOLTS = 60;
        public static final int BOTTOM_LAUNCHER_VOLTS = 50;
    }

    public static class Drivetrain{
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps;
        public static final double MAX_ANGULAR_RATE = 1.5 * Math.PI;
    }
}
