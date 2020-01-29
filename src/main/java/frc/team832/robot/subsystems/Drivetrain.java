package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.driverstation.dashboard.FalconDashboard;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.PDPPortNumber;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.sensors.NavXMicro;
import frc.team832.robot.Constants;

import frc.team832.robot.Robot;
import frc.team832.robot.accesories.ArcadeDriveProfile;
import frc.team832.robot.accesories.TankDriveProfile;

public class Drivetrain extends SubsystemBase implements DashboardUpdatable {
    private boolean initSuccessful = false;
    private CANTalonFX rightMaster, rightSlave, leftMaster, leftSlave;

    private SmartDiffDrive diffDrive;
    public DifferentialDriveOdometry driveOdometry;

    public Pose2d pose = new Pose2d();
    public NavXMicro navX;
    private Pose2d startingPose = new Pose2d();

    private double latestLeftWheelVolts, latestRightWheelVolts;

    private TankDriveProfile tankProfile = new TankDriveProfile();
    private ArcadeDriveProfile arcadeProfile = new ArcadeDriveProfile();

    private NetworkTable falconTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
    private NetworkTableEntry falconPoseXEntry = falconTable.getEntry("robotX");
    private NetworkTableEntry falconPoseYEntry = falconTable.getEntry("robotY");
    private NetworkTableEntry falconPoseHeadingEntry = falconTable.getEntry("robotHeading");
    private NetworkTableEntry falconIsPathingEntry = falconTable.getEntry("isFollowingPath");
    private NetworkTableEntry falconPathXEntry = falconTable.getEntry("pathX");
    private NetworkTableEntry falconPathYEntry = falconTable.getEntry("pathY");
    private NetworkTableEntry falconPathHeadingEntry = falconTable.getEntry("pathHeading");

    private SmartMCAttachedPDPSlot leftMasterSlot, leftSlaveSlot, rightMasterSlot, rightSlaveSlot;

    public Drivetrain(GrouchPDP pdp) {
        leftMaster = new CANTalonFX(Constants.DrivetrainValues.LEFT_MASTER_CAN_ID);
        leftSlave = new CANTalonFX(Constants.DrivetrainValues.LEFT_SLAVE_CAN_ID);
        rightMaster = new CANTalonFX(Constants.DrivetrainValues.RIGHT_MASTER_CAN_ID);
        rightSlave = new CANTalonFX(Constants.DrivetrainValues.RIGHT_SLAVE_CAN_ID);

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

        leftMasterSlot = pdp.addDevice(PDPPortNumber.Port0, leftMaster);
        leftSlaveSlot = pdp.addDevice(PDPPortNumber.Port0, leftSlave);
        rightMasterSlot = pdp.addDevice(PDPPortNumber.Port0, rightMaster);
        rightSlaveSlot = pdp.addDevice(PDPPortNumber.Port0, rightSlave);

        setCurrentLimit(40);

        diffDrive = new SmartDiffDrive(leftMaster, rightMaster, (int)Motor.kFalcon500.freeSpeed);
        driveOdometry = new DifferentialDriveOdometry(getDriveHeading(), startingPose);
        resetPose();

        setDefaultCommand(new RunEndCommand(this::tankDrive, this::stopDrive));

        initSuccessful = true;
    }

    public boolean isInitSuccessful() {
        return initSuccessful;
    }

    public void setCurrentLimit(int amps) {
        leftMaster.limitInputCurrent(amps);
        leftSlave.limitInputCurrent(amps);
        rightMaster.limitInputCurrent(amps);
        rightSlave.limitInputCurrent(amps);
    }

    public void tankDrive() {
        tankProfile.calculateTankSpeeds();
        diffDrive.tankDrive(tankProfile.leftPow, tankProfile.rightPow, tankProfile.loopMode);
    }

    public void arcadeDrive() {
        arcadeProfile.calculateArcadeSpeeds();
        diffDrive.arcadeDrive(arcadeProfile.xPow, arcadeProfile.rotPow, arcadeProfile.loopMode);
    }

    public void xBoxDrive() {
        arcadeProfile.calculateArcadeSpeeds();
        diffDrive.arcadeDrive(arcadeProfile.xPow, arcadeProfile.rotPow, arcadeProfile.loopMode);
    }

    public void stopDrive() {
        leftMaster.set(0);
        rightMaster.set(0);
    }

    public Rotation2d getDriveHeading() {
        return Rotation2d.fromDegrees(-navX.getYaw());
    }

    @Override
    public String getDashboardTabName () {
        return "Drivetrain";
    }

    @Override
    public void updateDashboardData () {
        FalconDashboard.updateRobotPose2d(pose);
    }

    public double getRightDistanceMeters () {
        return Constants.DrivetrainValues.DrivePowerTrain.calculateWheelDistanceMeters(-rightMaster.getSensorPosition());
    }

    public double getLeftDistanceMeters () {
        return Constants.DrivetrainValues.DrivePowerTrain.calculateWheelDistanceMeters(leftMaster.getSensorPosition());
    }

    public double getRightVelocityMetersPerSec () {
        return Constants.DrivetrainValues.DrivePowerTrain.calculateMetersPerSec(rightMaster.getSensorVelocity());
    }

    public double getLeftVelocityMetersPerSec () {
        return Constants.DrivetrainValues.DrivePowerTrain.calculateMetersPerSec(leftMaster.getSensorVelocity());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
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
        return pose;
    }

    private void updatePose() {
        pose = driveOdometry.update(getDriveHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
    }

    public void resetPose(Pose2d pose) {
        resetEncoders();
        this.pose = pose;
        navX.zero();
        driveOdometry.resetPosition(this.pose, getDriveHeading());
    }

    public void resetPose() {
        resetPose(startingPose);
    }

    public void resetEncoders() {
        leftMaster.rezeroSensor();
        rightMaster.rezeroSensor();
    }
}
