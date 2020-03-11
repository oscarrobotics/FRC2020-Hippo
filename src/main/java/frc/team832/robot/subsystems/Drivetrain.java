package frc.team832.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.driverstation.dashboard.FalconDashboard;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.util.MusicBox;
import frc.team832.robot.Constants;

import frc.team832.robot.Constants.DrivetrainValues;
import frc.team832.robot.utilities.ArcadeDriveProfile;
import frc.team832.robot.utilities.TankDriveProfile;

import java.util.Set;

@SuppressWarnings("SameParameterValue")
public class Drivetrain extends SubsystemBase implements DashboardUpdatable {
    public final boolean initSuccessful;
    private final CANTalonFX rightMaster, rightSlave, leftMaster, leftSlave;

    private final PigeonIMU imu;
    private final double[] ypr = new double[3];

    private final SmartDiffDrive diffDrive;
    private final DifferentialDriveOdometry driveOdometry;

    private Pose2d startingPose = Constants.FieldPositions.StartCenter;
    private Pose2d robotPose = startingPose;

    @SuppressWarnings("FieldCanBeLocal")
    private double latestLeftWheelVolts, latestRightWheelVolts;

    private final TankDriveProfile tankProfile = new TankDriveProfile(false, false);
    private final ArcadeDriveProfile arcadeProfile = new ArcadeDriveProfile();

    @SuppressWarnings("FieldCanBeLocal")
    private final SmartMCAttachedPDPSlot leftMasterSlot, leftSlaveSlot, rightMasterSlot, rightSlaveSlot;

    private final NetworkTableEntry dashboard_pigeonIMU_pitch, dashboard_pigeonIMU_roll, dashboard_pigeonIMU_fusedHeading,
            dashboard_poseX, dashboard_poseY, dashboard_poseRotation, ui_poseX, ui_poseY;

    private final CommandBase dashboardResetPoseCommand = new CommandBase() {
        @Override
        public Set<Subsystem> getRequirements() {
            return super.getRequirements();
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public void initialize() {
            Drivetrain.this.resetPoseFromDashboard();
        }
    };

    public final MusicBox driveMusic;

    public Drivetrain(GrouchPDP pdp) {
        leftMaster = new CANTalonFX(DrivetrainValues.LEFT_MASTER_CAN_ID);
        leftSlave = new CANTalonFX(DrivetrainValues.LEFT_SLAVE_CAN_ID);
        rightMaster = new CANTalonFX(DrivetrainValues.RIGHT_MASTER_CAN_ID);
        rightSlave = new CANTalonFX(DrivetrainValues.RIGHT_SLAVE_CAN_ID);

        driveMusic = new MusicBox(leftMaster, leftSlave, rightMaster, rightSlave);
        driveMusic.loadSong("Mario.chrp");

        leftMaster.wipeSettings();
        leftSlave.wipeSettings();
        rightMaster.wipeSettings();
        rightSlave.wipeSettings();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(false);
        leftSlave.setInverted(false);
        rightMaster.setInverted(false);
        leftSlave.setInverted(false);

        leftMasterSlot = pdp.addDevice(DrivetrainValues.LEFT_MASTER_PDP_PORT, leftMaster);
        leftSlaveSlot = pdp.addDevice(DrivetrainValues.LEFT_SLAVE_PDP_PORT, leftSlave);
        rightMasterSlot = pdp.addDevice(DrivetrainValues.RIGHT_MASTER_PDP_PORT, rightMaster);
        rightSlaveSlot = pdp.addDevice(DrivetrainValues.RIGHT_SLAVE_PDP_PORT, rightSlave);

        setNeutralMode(NeutralMode.kBrake);
        setCurrentLimit(40);
        DrivetrainValues.ClosedLoopDT.setFFAccel(0.1);

        imu = new PigeonIMU(0);

        diffDrive = new SmartDiffDrive(leftMaster, rightMaster, DrivetrainValues.ClosedLoopDT, DrivetrainValues.MaxRpm);
        driveOdometry = new DifferentialDriveOdometry(getDriveHeading(), startingPose);
        resetPose();

        DashboardManager.addTab(this, this);
        dashboard_pigeonIMU_pitch = DashboardManager.addTabItem(this, "IMU/Pitch", 0.0);
        dashboard_pigeonIMU_roll = DashboardManager.addTabItem(this, "IMU/Roll", 0.0);
        dashboard_pigeonIMU_fusedHeading = DashboardManager.addTabItem(this, "IMU/FusedHeading", 0.0);
        dashboard_poseX = DashboardManager.addTabItem(this, "Pose/X", 0.0);
        dashboard_poseY = DashboardManager.addTabItem(this, "Pose/Y", 0.0);
        dashboard_poseRotation = DashboardManager.addTabItem(this, "Pose/Rotation", 0.0);
        ui_poseX = DashboardManager.addTabItem(this, "Starting Pose X", startingPose.getTranslation().getX());
        ui_poseY = DashboardManager.addTabItem(this, "Starting Pose Y", startingPose.getTranslation().getY());

        DashboardManager.getTab(this).add("Pose", dashboardResetPoseCommand);

//        DashboardManager.addTabButton(this, "Reset Pose", this::resetPoseFromDashboard);

        setDefaultCommand(new RunEndCommand(this::tankDrive, this::stopDrive, this));

        initSuccessful = leftMaster.getCANConnection() && leftSlave.getCANConnection() &&
                rightMaster.getCANConnection() && rightSlave.getCANConnection() && imu.getState() != PigeonIMU.PigeonState.NoComm;
    }

    @Override
    public void periodic() {
        updatePose();
    }

    private void tankDrive() {
        tankProfile.calculateTankSpeeds();
        diffDrive.tankDrive(tankProfile.leftPower, tankProfile.rightPower, tankProfile.loopMode);
    }

    @SuppressWarnings("unused")
    public void arcadeDrive() {
        arcadeProfile.calculateArcadeSpeeds();
        diffDrive.arcadeDrive(arcadeProfile.xPow, arcadeProfile.rotPow, arcadeProfile.loopMode);
    }

    @SuppressWarnings("unused")
    public void xBoxDrive() {
        arcadeProfile.calculateArcadeSpeeds();
        diffDrive.arcadeDrive(arcadeProfile.xPow, arcadeProfile.rotPow, arcadeProfile.loopMode);
    }

    private void stopDrive() {
        leftMaster.set(0);
        rightMaster.set(0);
    }

    private Rotation2d getDriveHeading() {
        return Rotation2d.fromDegrees(imu.getFusedHeading());
    }

    @Override
    public void updateDashboardData() {
        FalconDashboard.updateRobotPose2d(robotPose);
        imu.getYawPitchRoll(ypr);
        dashboard_pigeonIMU_pitch.setDouble(ypr[1]);
        dashboard_pigeonIMU_roll.setDouble(ypr[2]);
        dashboard_pigeonIMU_fusedHeading.setDouble(imu.getFusedHeading());
        dashboard_poseX.setDouble(robotPose.getTranslation().getX());
        dashboard_poseY.setDouble(robotPose.getTranslation().getY());
        dashboard_poseRotation.setDouble(robotPose.getRotation().getDegrees());
    }

    private double getRightDistanceMeters() {
        return DrivetrainValues.DrivePowerTrain.calculateWheelDistanceMeters(-rightMaster.getSensorPosition());
    }

    private double getLeftDistanceMeters() {
        return DrivetrainValues.DrivePowerTrain.calculateWheelDistanceMeters(leftMaster.getSensorPosition());
    }

    private double getRightVelocityMetersPerSec() {
        return DrivetrainValues.DrivePowerTrain.calculateMetersPerSec(rightMaster.getSensorVelocity());
    }

    private double getLeftVelocityMetersPerSec() {
        return DrivetrainValues.DrivePowerTrain.calculateMetersPerSec(leftMaster.getSensorVelocity());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());
    }

    public void setWheelVolts(Double leftVolts, Double rightVolts) {
        double leftBusVoltage = leftMaster.getInputVoltage();
        double rightBusVoltage = rightMaster.getInputVoltage();

        latestLeftWheelVolts = Math.abs(leftVolts / leftBusVoltage) * Math.signum(leftVolts);
        latestRightWheelVolts = -Math.abs(rightVolts / rightBusVoltage) * Math.signum(rightVolts);

        leftMaster.set(latestLeftWheelVolts);
        rightMaster.set(latestRightWheelVolts);
    }

    public Pose2d getLatestPose() {
        updatePose();
        return robotPose;
    }

    private void updatePose() {
        robotPose = driveOdometry.update(getDriveHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    }

    public void resetPose(double x, double y) {
        resetPose(new Pose2d(x, y, getDriveHeading()));
    }

    private void resetPose(Pose2d pose) {
        resetEncoders();
        zeroYaw();
        robotPose = pose;
        driveOdometry.resetPosition(robotPose, getDriveHeading());
    }

    private void resetPoseFromDashboard() {
        System.out.println("fugg");
        Pose2d pose = new Pose2d(ui_poseX.getDouble(startingPose.getTranslation().getX()), ui_poseY.getDouble(startingPose.getTranslation().getY()), getDriveHeading());
        resetPose(pose);
    }

    private void resetPose() {
        resetPose(startingPose);
    }

    private void resetEncoders() {
        leftMaster.rezeroSensor();
        rightMaster.rezeroSensor();
    }

    private void zeroYaw() {
        imu.setFusedHeading(0);
    }

    public void setNeutralMode(NeutralMode mode) {
        leftMaster.setNeutralMode(mode);
        leftSlave.setNeutralMode(mode);
        rightMaster.setNeutralMode(mode);
        rightSlave.setNeutralMode(mode);
    }

    private void setCurrentLimit(int amps) {
        leftMaster.limitInputCurrent(amps);
        leftSlave.limitInputCurrent(amps);
        rightMaster.limitInputCurrent(amps);
        rightSlave.limitInputCurrent(amps);
    }

    @Override
    public String getDashboardTabName() { return "Drivetrain"; }

}
