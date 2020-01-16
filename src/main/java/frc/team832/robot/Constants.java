package frc.team832.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.team832.lib.motors.DTPowerTrain;
import frc.team832.lib.motors.Gearbox;
import frc.team832.lib.motors.Motor;

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

        public static final double kDriveWheelDiameter = Units.inchesToMeters(6);

        public static final float kDriveGearReduction = 1f / (9f/84f);

        private static final Gearbox driveGearbox = new Gearbox(kDriveGearReduction);
        public static final DTPowerTrain dtPowertrain = new DTPowerTrain(driveGearbox, Motor.kFalcon500, 2, kDriveWheelDiameter);
        public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28.0));

        private static final double kDrive_kS = 0.0;
        private static final double kDrive_kV = 0.0;
        private static final double kDrive_kA = 0.0;

        private static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward(kDrive_kS, kDrive_kV, kDrive_kA);

        public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(kDriveFF, kDriveKinematics, 10);

        public static final TrajectoryConfig kTrajectoryConfig =
                new TrajectoryConfig(2, 4)
                        .setKinematics(kDriveKinematics)
                        .addConstraint(kAutoVoltageConstraint);
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

    public static class SpindexerValues {
        public static final int SPIN_MOTOR_CAN_ID = 6;
        public static final int FEED_MOTOR_CAN_ID = 7;
    }






}
