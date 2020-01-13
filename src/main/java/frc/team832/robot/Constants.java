package frc.team832.robot;

public class Constants {
    public static class DrivetrainValues {
        public static final int RIGHT_MASTER_CAN_ID = 1;
        public static final int LEFT_MASTER_CAN_ID = 2;
        public static final int RIGHT_SLAVE_CAN_ID = 3;
        public static final int LEFT_SLAVE_CAN_ID = 4;

        public static final double stickDriveMultiplier = 1.0;
        public static final double stickRotateOnCenterMultiplier = 0.6;
        public static final double stickRotateMultiplier = 0.85;

        public static final int MAX_RPM = 5000;

    }

    public static class ShooterValues {
        public static final int SHOOTER_ID_PRIMARY = 6;
        public static final int SHOOTER_ID_SECONDARY = 7;

        public static double SPIN_UP_kP = 0;
        public static double SPIN_UP_kD = 0;
        public static double SPIN_UP_kF = 0;

        public static double SHOOTING_kP = 0;
        public static double SHOOTING_kD = 0;
        public static double SHOOTING_kF = 0;

        public static double SPIN_DOWN_kP = 0;
        public static double SPIN_DOWN_kD = 0;
        public static double SPIN_DOWN_kF = 0;

        public static double IDLE_kP = 0;
        public static double IDLE_kD = 0;
        public static double IDLE_kF = 0;

    }

    public static class IntakeValues {
        public static final int INTAKE_MOTOR_CAN_ID = 5;

    }






}
