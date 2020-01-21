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

        public static final double kLeft_kP = .00;
        public static final double kRight_kP = .00;

        public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward(kDrive_kS, kDrive_kV, kDrive_kA);

        public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(kDriveFF, kDriveKinematics, 10);

        public static final TrajectoryConfig kTrajectoryConfig =
                new TrajectoryConfig(2, 4)
                        .setKinematics(kDriveKinematics)
                        .addConstraint(kAutoVoltageConstraint);
    }

    public static class ShooterValues {
        public static final int PRIMARY_CAN_ID = 6;
        public static final int SECONDARY_CAN_ID = 7;

        public static final int HOOD_CAN_ID = 8;
        public static final int TURRET_CAN_ID = 9;

        public static final double TURRET_kP = 0;
        public static final double TURRET_kD = 0;
        public static final double TURRET_kF = 0;

        public static final double HOOD_kP = 0;
        public static final double HOOD_kD = 0;
        public static final double HOOD_kF = 0;

        public static final double SPIN_UP_kP = 0;
        public static final double SPIN_UP_kD = 0;
        public static final double SPIN_UP_kF = 0;

        public static final double SHOOTING_kP = 0;
        public static final double SHOOTING_kD = 0;
        public static final double SHOOTING_kF = 0;

        public static final double SPIN_DOWN_kP = 0;
        public static final double SPIN_DOWN_kD = 0;
        public static final double SPIN_DOWN_kF = 0;

        public static final double IDLE_kP = 0;
        public static final double IDLE_kD = 0;
        public static final double IDLE_kF = 0;

    }

    public static class IntakeValues {
        public static final int INTAKE_MOTOR_CAN_ID = 5;

    }

    public static class SpindexerValues {
        public static final int SPIN_MOTOR_CAN_ID = 10;
        public static final int FEED_MOTOR_CAN_ID = 11;

        public static final int HALL_EFFECT_CHANNEL = 0;
    }

    public static class ClimberValues {
        public static final int WINCH_CAN_ID = 12;
        public static final double WINCH_POWER = .5;
    }

    public static class WOFValues {
        public static final int SPINNER_CAN_ID = 13;
        public static final double RevsToTicks = .0002;// 5000 encoder ticks = 1 color wheel revolution (just a guess)
    }

    public class PneumaticsValues {
        public static final int INTAKE_SOLENOID_ID = 0;
        public static final int PROP_UP_SOLENOID_ID = 0;
        public static final int PCM_MODULE_NUM = 0;
        public static final int WHEEL_O_FORTUNE_SOLENOID_ID = 0;
        public static final int CLIMB_LOCK_SOLENOID_ID = 0;
    }
}
