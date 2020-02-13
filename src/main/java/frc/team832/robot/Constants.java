package frc.team832.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.team832.lib.motors.WheeledPowerTrain;
import frc.team832.lib.motors.Gearbox;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.PDPPortNumber;

public class Constants {
    public static class DrivetrainValues {
        public static final int LEFT_MASTER_CAN_ID = 1;
        public static final int LEFT_SLAVE_CAN_ID = 2;
        public static final int RIGHT_MASTER_CAN_ID = 3;
        public static final int RIGHT_SLAVE_CAN_ID = 4;

        public static final PDPPortNumber LEFT_MASTER_PDP_PORT = PDPPortNumber.Port15;
        public static final PDPPortNumber LEFT_SLAVE_PDP_PORT = PDPPortNumber.Port14;
        public static final PDPPortNumber RIGHT_MASTER_PDP_PORT = PDPPortNumber.Port0;
        public static final PDPPortNumber RIGHT_SLAVE_PDP_PORT = PDPPortNumber.Port1;

        public static final double StickDriveMultiplier = 1.0;
        public static final double StickRotateOnCenterMultiplier = 0.6;
        public static final double StickRotateMultiplier = 0.8;

        public static final double DriveWheelDiameter = Units.inchesToMeters(6);
        public static final float DriveGearReduction = 1f / (8f/84f);

        public static final int MaxRpm = (int)Motor.kFalcon500.freeSpeed;

        private static final Gearbox DriveGearbox = new Gearbox(DriveGearReduction);
        public static final WheeledPowerTrain DrivePowerTrain = new WheeledPowerTrain(DriveGearbox, Motor.kFalcon500, 2, DriveWheelDiameter);
        public static DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(29.0));

        public static final double LeftkP = 0.1;
        public static final double LeftkD = 0.01;

        public static final double RightkP = 0.1;
        public static final double RightkD = 0.01;

        public static final SimpleMotorFeedforward LeftFF = new SimpleMotorFeedforward(0, 0, 0);
        public static final SimpleMotorFeedforward RightFF = new SimpleMotorFeedforward(0, 0, 0);

        public static final DifferentialDriveVoltageConstraint LeftAutoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(LeftFF, DriveKinematics, 10);
        public static final DifferentialDriveVoltageConstraint RightAutoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(RightFF, DriveKinematics, 10);

        private static double Velocity = 2, Acceleration = 4;
        public static final TrajectoryConfig LeftTrajectoryConfig =
                new TrajectoryConfig(Velocity, Acceleration)
                        .setKinematics(DriveKinematics)
                        .addConstraint(LeftAutoVoltageConstraint);
        public static final TrajectoryConfig RightTrajectoryConfig =
                new TrajectoryConfig(Velocity, Acceleration)
                        .setKinematics(DriveKinematics)
                        .addConstraint(RightAutoVoltageConstraint);
    }

    public static class ShooterValues {
        public static final int PRIMARY_CAN_ID = 2;
        public static final int SECONDARY_CAN_ID = 3;
        public static final int TURRET_CAN_ID = 4;
        public static final int FEED_MOTOR_CAN_ID = 6;

        public static final PDPPortNumber PRIMARY_PDP_SLOT = PDPPortNumber.Port2;
        public static final PDPPortNumber SECONDARY_PDP_SLOT = PDPPortNumber.Port3;
        public static final PDPPortNumber TURRET_PDP_SLOT = PDPPortNumber.Port4;
        public static final PDPPortNumber FEEDER_PDP_SLOT = PDPPortNumber.Port6;

        public static final int HOOD_CHANNEL = 0;

        public static final float FlywheelReduction = 2f / 1f;
        private static final Gearbox FlywheelGearbox = new Gearbox(FlywheelReduction);
        public static final WheeledPowerTrain FlywheelPowerTrain = new WheeledPowerTrain(FlywheelGearbox, Motor.kNEO, 2, Units.inchesToMeters(4));

        public static final float TurretReduction = 1f / (200f/1f);
        private static final Gearbox TurretGearbox = new Gearbox(TurretReduction);
        public static final WheeledPowerTrain TurretPowerTrain = new WheeledPowerTrain(TurretGearbox, Motor.kNEO550, 1, Units.inchesToMeters(10));

        public static final float FeedReduction = 1f;
        private static final Gearbox FeedGearbox = new Gearbox(FeedReduction);
        public static final WheeledPowerTrain FeedPowertrain = new WheeledPowerTrain(FeedGearbox, Motor.kNEO, 1, Units.inchesToMeters(4));

        public static final double FeedRpm = FeedPowertrain.calculateMotorRpmFromWheelRpm(3000);

        public static final double TURRET_kP = 0;
        public static final double TURRET_kD = 0;

        public static final Constraints TURRET_CONSTRAINTS = new Constraints(TurretPowerTrain.calculateMotorRpmFromWheelRpm(90),
                                                                            TurretPowerTrain.calculateMotorRpmFromWheelRpm(450));

        public static final double HOOD_kP = 0;
        public static final double HOOD_kD = 0;
        public static final double HOOD_kF = 0;

        public static final double SPIN_UP_kP = 0;
        public static final double SPIN_UP_kD = 0;
        public static final double SPIN_UP_kF = 0;

        public static final double SHOOTING_kP = 0;
        public static final double SHOOTING_kD = 0;
        public static final double SHOOTING_kF = 0;

        public static final double IDLE_kP = 0;
        public static final double IDLE_kD = 0;
        public static final double IDLE_kF = 0;
    }

    public static class IntakeValues {
        public static final int INTAKE_MOTOR_CAN_ID = 1;

        public static final PDPPortNumber INTAKE_MOTOR_PDP_SLOT = PDPPortNumber.Port11;

        public static final float IntakeReduction = 1f / (18f/36f);
        private static final Gearbox IntakeGearbox = new Gearbox(IntakeReduction);
        public static final WheeledPowerTrain IntakePowertrain = new WheeledPowerTrain(IntakeGearbox, Motor.kNEO550, 1, Units.inchesToMeters(2));

        private static final double kS = 0.25;
        private static final double kV = 0.0005;
        private static final double kA = 0.0001;

        public static final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(kS, kV, kA);

    }

    public static class SpindexerValues {
        public static final int SPIN_MOTOR_CAN_ID = 5;

        public static final PDPPortNumber SPIN_MOTOR_PDP_SLOT = PDPPortNumber.Port5;

        public static final int HALL_EFFECT_CHANNEL = 0;

        public static final float SpinReduction = 1f / (56f/1f);
        private static final Gearbox SpinGearbox = new Gearbox(SpinReduction);
        public static final WheeledPowerTrain SpinPowertrain = new WheeledPowerTrain(SpinGearbox, Motor.kNEO, 1, Units.inchesToMeters(20));

        public static final double FEED_kP = 0;
        public static final double FEED_kD = 0;
        public static final double FEED_kF = 0;

        public static final double SPIN_kP = 0;
        public static final double SPIN_kD = 0;
        public static final double SPIN_kF = 0;

        public static Constraints Constraints = new Constraints(2, 8);

        public static final double POSITION_kP = 0;
        public static final double POSITION_kD = 0;
        public static final double POSITION_kF = 0;
    }

    public static class ClimberValues {
        public static final int WINCH_CAN_ID = 7;
        public static final int DEPLOY_CAN_ID = 8;

        public static final PDPPortNumber WINCH_PDP_PORT = PDPPortNumber.Port13;
        public static final PDPPortNumber DEPLOY_PDP_PORT = PDPPortNumber.Port14;

    }

    public static class WOFValues {
        public static final int SPINNER_CAN_ID = 9;

        public static final float SpinReduction = 1f / (25f/1f);
        private static final Gearbox SpinGearbox = new Gearbox(SpinReduction);
        public static final WheeledPowerTrain SpinPowertrain = new WheeledPowerTrain(SpinGearbox, Motor.kNEO550, 1, Units.inchesToMeters(4));
    }

    public class PneumaticsValues {
        public static final int PCM_MODULE_NUM = 0;

        public static final int INTAKE_SOLENOID_ID = 1;
        public static final int PROP_UP_SOLENOID_ID = 2;
        public static final int WHEEL_O_FORTUNE_SOLENOID_ID = 3;
        public static final int CLIMB_LOCK_SOLENOID_ID = 4;
    }

    public static class FieldPositions {
        // positions are relative to driver station
        public static final Pose2d StartCenter = new Pose2d(11.875, 3.05, Rotation2d.fromDegrees(0));
        public static final Pose2d CloseSideTrench = new Pose2d(5.245, 7.505, Rotation2d.fromDegrees(0));
        public static final Pose2d FarSideTrench = new Pose2d(10.735, 7.505, Rotation2d.fromDegrees(0));
        public static final Pose2d ShieldGenCloseToTrench = new Pose2d(10.735-.5, 5.748, Rotation2d.fromDegrees(270)); // Needs to be changed
    }

    public class LEDValues {
        public static final int LED_PWM_PORT = 9;
    }
}
