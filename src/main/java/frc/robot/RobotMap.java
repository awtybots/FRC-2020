package frc.robot;

public final class RobotMap {

    public final static class MotorIDs {
        public static final int DRIVE_L1 = 1;
        public static final int DRIVE_L2 = 2;

        public static final int DRIVE_R1 = 3;
        public static final int DRIVE_R2 = 4;

        public static final int INTAKE = 9;

        public static final int INDEXER_L = 8;
        public static final int INDEXER_R = 7;

        public static final int TOWER_1 = 5;
        public static final int TOWER_2 = 6;

        public static final int FLYWHEEL = 10;

        public static final int CONTROL_PANEL_SPINNER = 13;
    }

    public final static class SolenoidChannels {
        public static final int INTAKE_FWD = 0;
        public static final int INTAKE_REV = 4;

        public static final int CLIMB_FWD = 1;
        public static final int CLIMB_REV = 2;

        public static final int ANGLE_CLIMB_FWD = 6;
        public static final int ANGLE_CLIMB_REV = 7;
    }

}
