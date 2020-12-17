package frc.team832.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.team832.lib.drive.ClosedLoopDT;
import frc.team832.lib.motors.WheeledPowerTrain;
import frc.team832.lib.motors.Gearbox;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.PDPPortNumber;
import frc.team832.lib.util.ClosedLoopConfig;
import frc.team832.lib.util.OscarMath;


@SuppressWarnings({"WeakerAccess", "unused"})
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

        public static final double StickDriveMultiplier = 1;
        public static final double StickRotateOnCenterMultiplier = 0.6;

        public static final double DriveWheelDiameter = 6 * 0.0254;
        public static final float DriveGearReduction = 1f / (8f/84f);

        public static final int MaxRpm = (int)Motor.kFalcon500.freeSpeed;

        private static final Gearbox DriveGearbox = new Gearbox(DriveGearReduction);
        public static final WheeledPowerTrain DrivePowerTrain = new WheeledPowerTrain(DriveGearbox, Motor.kFalcon500, 2, DriveWheelDiameter);
        public static DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(0.6763419203747071);

        public static final SimpleMotorFeedforward CombinedFF = new SimpleMotorFeedforward(0.115, 2.33, 0.165);
        public static final SimpleMotorFeedforward LeftFF = new SimpleMotorFeedforward(0.109, 2.34, 0.165);
        public static final SimpleMotorFeedforward RightFF = new SimpleMotorFeedforward(0.121, 2.31, 0.165);

        public static final ClosedLoopConfig LeftConfig = new ClosedLoopConfig(0.01, 0, 0.001, 0);
        public static final ClosedLoopConfig RightConfig = new ClosedLoopConfig(0.01, 0, 0.001, 0);

        public static final ClosedLoopDT ClosedLoopDT = new ClosedLoopDT(LeftFF, RightFF, LeftConfig, RightConfig, DrivePowerTrain);

        public static final double LeftkP = 0.0001;
        public static final double LeftkD = 0.0;

        public static final double RightkP = 0.0001;
        public static final double RightkD = 0.0;

        public static final DifferentialDriveVoltageConstraint LeftAutoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(LeftFF, DriveKinematics, 10);
        public static final DifferentialDriveVoltageConstraint RightAutoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(RightFF, DriveKinematics, 10);

        private static double Velocity = 1, Acceleration = 1;
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
        public static final int FEED_MOTOR_CAN_ID = 6;

        public static final PDPPortNumber PRIMARY_PDP_SLOT = PDPPortNumber.Port2;
        public static final PDPPortNumber SECONDARY_PDP_SLOT = PDPPortNumber.Port3;
        public static final PDPPortNumber FEEDER_PDP_SLOT = PDPPortNumber.Port6;

        //Shooter
        public static final int HOOD_SERVO_PWM_CHANNEL = 0;
        public static final int HOOD_POTENTIOMETER_ANALOG_CHANNEL = 0;

        public static final int FLYWHEEL_ENCODER_DIO_CHANNEL_A = 3;
        public static final int FLYWHEEL_ENCODER_DIO_CHANNEL_B = 4;


        public static final float FlywheelReduction = 26f / 50f;

        private static final double FlywheelkS = 0.0437;
        private static final double FlywheelkV = 0.00217;
        private static final double FlywheelkA = 0.00103;

        public static final ClosedLoopConfig ShootingConfig = new ClosedLoopConfig(0.0004, 0.0, 0.000001, 0.0);

        public static final SimpleMotorFeedforward FlywheelFF = new SimpleMotorFeedforward(FlywheelkS, FlywheelkV, FlywheelkA);

        public static final double FlywheelMOI = 0.00179;
//        public static final double FlywheelMOI = 0.100895;

        public static final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(2),
                FlywheelMOI,
                FlywheelReduction);

        public static final double ControlLoopPeriod = 0.01;

        //Hood
        public static final double HoodReduction = 34.0 / 340.0;

        public static final double HoodkP = 8.0;

        public static final double HoodMaxAngle = 70;
        public static final double HoodMinAngle = 10;

        public static final double HoodBottom = 4.57;
        public static final double HoodTop = 1.86;


        //Feeder
        public static final double FeedRpm = 3000;

        public static final double FeedkP = 0.00015;
        public static final double FeedkF = 0;

        private static final double FeederkS = 0.0;
        private static final double FeederkV = 0.0;
        private static final double FeederkA = 0.0;

        public static final SimpleMotorFeedforward FeederFF = new SimpleMotorFeedforward(FeederkS, FeederkV, FeederkA);

    }

    public static class TurretValues {
        public static final int TURRET_MOTOR_CAN_ID = 4;
        public static final int TURRET_ENCODER_DIO_CHANNEL = 0;

        public static final PDPPortNumber TURRET_PDP_SLOT = PDPPortNumber.Port4;

        public static final double PracticeTurretLeftPosition = -83;
        public static final double PracticeTurretRightPosition = 83;

        public static final int VisionTargetingRange = 120;
        public static final int PracticeTurretLeftVisionPosition = -(VisionTargetingRange / 2);
        public static final int PracticeTurretRightVisionPosition = VisionTargetingRange / 2;
        public static final int TurretCenterVisionPosition = 0;

        public static final TrapezoidProfile.Constraints SSConstraints = new TrapezoidProfile.Constraints(
                Units.degreesToRadians(360), Units.degreesToRadians(1440)); // Max arm speed and acceleration.

        public static final double TurretMOI = 0.068185353;

        public static final double TurretReduction = 200;//200:1

        public static final LinearSystem<N2, N1, N1> m_turretPlant = LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getNeo550(1),
            TurretMOI,
            TurretReduction
        );

        public static final double kP = 0.015;
        public static final double kI = 0.75;
        public static final double kD = 0.000;
        public static final double FFMultiplier = 0.001;

        public static final double IntakeOrientationDegrees = -90;

        public static double ControlLoopPeriod = 0.005;

        public static double convertRotationsToDegrees(double rotations) {
            return OscarMath.map(rotations, 0, 1, -180, 180);
        }

        public static double convertDegreesToRotation(double degrees){
            return OscarMath.map(degrees, -180, 180, 0,1);
        }


    }

    public static class IntakeValues {
        public static final int INTAKE_MOTOR_CAN_ID = 5;

        public static final PDPPortNumber INTAKE_MOTOR_PDP_SLOT = PDPPortNumber.Port11;

        public static final float IntakeReduction = 1f / (36f/18f);

        private static final double kS = 0.01;
        private static final double kV = Motor.kNEO550.kv;
        private static final double kA = 0.001;

        public static final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public static class SpindexerValues {
        public static final int SPIN_MOTOR_CAN_ID = 5;

        public static final PDPPortNumber SPIN_MOTOR_PDP_SLOT = PDPPortNumber.Port5;

        public static final int HALL_EFFECT_DIO_CHANNEL = 1;
        public static final int LASERSHARK_DIO_CHANNEL = 2;

        public static final float SpinReduction = 1f / 56f;
        private static final Gearbox SpinGearbox = new Gearbox(SpinReduction);
        public static final WheeledPowerTrain SpinPowertrain = new WheeledPowerTrain(SpinGearbox, Motor.kNEO, 1, Units.inchesToMeters(20));

        public static final double SpinkP = 0.01;
        public static final double SpinkD = 0.0;
        public static Constraints   VelocityConstraints  = new Constraints(SpinPowertrain.calculateMotorRpmFromWheelRpm(500), SpinPowertrain.calculateMotorRpmFromWheelRpm(500));

        public static final double PositionkP = 2.0;
        public static Constraints PositionConstraints = new Constraints(SpinPowertrain.calculateMotorRpmFromWheelRpm(90), SpinPowertrain.calculateMotorRpmFromWheelRpm(180));

    }

    public static class ClimberValues {
        public static final int WINCH_CAN_ID = 7;
        public static final int DEPLOY_CAN_ID = 8;

        public static final PDPPortNumber WINCH_PDP_PORT = PDPPortNumber.Port13;
        public static final PDPPortNumber DEPLOY_PDP_PORT = PDPPortNumber.Port11;

        public static final float ExtendReduction = 1f / (5f/1f);

        public static final double MaxExtend = -62;
        public static final double MinExtend = -40;
        public static final double Retract = -0.25;

        public static final Constraints ExtendConstraints = new Constraints(120, 480);
        public static final Constraints ClimbConstraints = new Constraints(10, 20);
        //velocity might be acceleration and acceleration might be jerk because PID is running on velocity and not position

        public static final double ExtendkP = 0.08;
        public static final double ClimbkP = 0.01;

        public static double ClimbVelocity = 10;
    }

    public static class WOFValues {
        public static final int SPINNER_CAN_ID = 9;

        public static final float SpinReduction = 1f / 25f;
        private static final Gearbox SpinGearbox = new Gearbox(SpinReduction);
        public static final WheeledPowerTrain SpinPowertrain = new WheeledPowerTrain(SpinGearbox, Motor.kNEO550, 1, Units.inchesToMeters(4));

        public static Constraints Constraints = new Constraints(4, 12);

        public static final double kP = 0.0;
        public static final double kF = 0.0;
    }

    public static class PneumaticsValues {
        public static final int PCM_MODULE_NUM = 0;

        public static final int INTAKE_SOLENOID_ID = 0;
        public static final int PROP_UP_SOLENOID_ID = 1;
        public static final int WHEEL_O_FORTUNE_SOLENOID_ID = 2;
        public static final int CLIMB_LOCK_SOLENOID_ID = 3;
    }

    public enum FieldPosition {
        // positions are relative to driver station
        ZeroZero(0, 0, 0),
        InitLine_CenteredOnPort(Units.feetToMeters(10.75), Units.feetToMeters(19), 180),
        StartCenter(11.875, 3.05, 180),
        CloseSideTrench(5.245, 7.505, 180),
        FarSideTrench(10.735, 7.505, 0),
        ShieldGenCloseToTrench(10.235, 5.748, 270); // Needs to be changed

        public final Pose2d poseMeters;

        FieldPosition(double xMeters, double yMeters, double degrees) {
            poseMeters = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(degrees));
        }
    }

    public static class LEDValues {
        public static final int LED_PWM_PORT = 9;
    }
}
