package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.sensors.NavXMicro;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

import frc.team832.robot.accesories.ArcadeDriveProfile;
import frc.team832.robot.accesories.TankDriveProfile;
import frc.team832.robot.commands.TemplateCommand;

import static frc.team832.robot.Robot.oi;

public class Drivetrain extends SubsystemBase implements DashboardUpdatable {
    private boolean initSuccessful = false;
    private CANTalonFX rightMaster, rightSlave, leftMaster, leftSlave;

    private SmartDiffDrive diffDrive;
    public DifferentialDriveOdometry driveOdometry;

    public Pose2d pose = new Pose2d();
    public NavXMicro navX;
    private Pose2d startingPose = new Pose2d();

    private TankDriveProfile tankProfile = new TankDriveProfile();
    ArcadeDriveProfile arcadeProfile = new ArcadeDriveProfile();

    private NetworkTable falconTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
    private NetworkTableEntry falconPoseXEntry = falconTable.getEntry("robotX");
    private NetworkTableEntry falconPoseYEntry = falconTable.getEntry("robotY");
    private NetworkTableEntry falconPoseHeadingEntry = falconTable.getEntry("robotHeading");
    private NetworkTableEntry falconIsPathingEntry = falconTable.getEntry("isFollowingPath");
    private NetworkTableEntry falconPathXEntry = falconTable.getEntry("pathX");
    private NetworkTableEntry falconPathYEntry = falconTable.getEntry("pathY");
    private NetworkTableEntry falconPathHeadingEntry = falconTable.getEntry("pathHeading");

    public Drivetrain() {
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

        setCurrentLimit(40);

        diffDrive = new SmartDiffDrive(leftMaster, rightMaster, Constants.DrivetrainValues.MAX_RPM);
        driveOdometry = new DifferentialDriveOdometry(getDriveHeading(), startingPose);
        resetPose();

        setDefaultCommand(new TemplateCommand(this));

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

    public ArcadeDriveProfile calculateArcadeSpeeds() {
        double xPower = 0;
        double rotPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getArcadeDriveAxes();
        xPower = OscarMath.signumPow(-axes.getRight() * Constants.DrivetrainValues.stickDriveMultiplier, 3);
        rotPower = OscarMath.signumPow(axes.getLeft() * Constants.DrivetrainValues.stickDriveMultiplier, 3);

        return new ArcadeDriveProfile(xPower, rotPower, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public void tankDrive() {
        tankProfile.calculateTankSpeeds();
        diffDrive.tankDrive(tankProfile.leftPow, tankProfile.rightPow, tankProfile.loopMode);
    }

    public void xBoxDrive() {
        arcadeProfile.calculateArcadeSpeeds();
        diffDrive.arcadeDrive(arcadeProfile.xPow, arcadeProfile.rotPow, arcadeProfile.loopMode);
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

    }

    public void updateDashboardPose() {
        var translation = pose.getTranslation();
        var poseX = translation.getX();
        var poseY = translation.getY();
        var heading = pose.getRotation();

        falconPoseXEntry.setDouble(Units.metersToFeet(poseX));
        falconPoseYEntry.setDouble(Units.metersToFeet(poseY));
        falconPoseHeadingEntry.setDouble(heading.getRadians());
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
