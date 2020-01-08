package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;

import frc.team832.robot.accesories.ArcadeDriveProfile;
import frc.team832.robot.accesories.TankDriveProfile;
import frc.team832.robot.commands.TemplateCommand;

import static frc.team832.robot.Robot.oi;

public class Drivetrain extends SubsystemBase {
    private boolean initSuccessful = false;
    private CANTalonFX rightMaster, rightSlave, leftMaster, leftSlave;
    private SmartDiffDrive diffDrive;

    private final double stickDriveMultiplier = 1.0;
    private final double stickRotateOnCenterMultiplier = 0.6;
    private final double stickRotateMultiplier = 0.85;

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

    public void arcadeDrive() {
        double rightPower = 0;
        double leftPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getArcadeDriveAxes();
        rightPower = OscarMath.signumPow(-axes.getRight() * stickDriveMultiplier, 3);
        leftPower = OscarMath.signumPow(axes.getLeft() * stickDriveMultiplier, 3);
        diffDrive.arcadeDrive(rightPower, leftPower, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public TankDriveProfile calculateTankSpeeds() {
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        boolean isRotate = ((SticksDriverOI) oi.driverOI).rightStick.two.get();
        boolean driveStraight = ((SticksDriverOI) oi.driverOI).leftStick.trigger.get() || ((SticksDriverOI) oi.driverOI).rightStick.trigger.get();

        if (driveStraight) {
            TankDriveProfile straight = getTankStraightProfile();
            if (isRotate) {
                TankDriveProfile rotate = getTankRotateProfile();
                return new TankDriveProfile(straight.leftPow + rotate.leftPow, straight.rightPow + rotate.rightPow, SmartDiffDrive.LoopMode.PERCENTAGE);
            } else {
                return straight;
            }
        } else {
            if (isRotate) {
                return getTankRotateOnCenterProfile();

            } else {
                return getTankNormalProfile();
            }
        }
    }

    public TankDriveProfile getTankStraightProfile() {
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        double rightStick = axes.getRight();
        double leftStick = axes.getLeft();
        double power;
        if (((SticksDriverOI) oi.driverOI).leftStick.trigger.get() && ((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
            power = (OscarMath.signumPow(rightStick * stickDriveMultiplier, 2) + OscarMath.signumPow(leftStick * stickDriveMultiplier, 2)) / 2;
        } else if (((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
            power = OscarMath.signumPow(rightStick * stickDriveMultiplier, 2);
        } else {
            power = (OscarMath.signumPow(leftStick * stickDriveMultiplier, 2));
        }

        return new TankDriveProfile(power, power, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public TankDriveProfile getTankNormalProfile() {
        double rightPower = 0;
        double leftPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        double rightStick = axes.getRight();
        double leftStick = axes.getLeft();

        rightPower = OscarMath.signumPow(rightStick * stickDriveMultiplier, 2);
        leftPower = OscarMath.signumPow(leftStick * stickDriveMultiplier, 2);

        return new TankDriveProfile(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public TankDriveProfile getTankRotateProfile() {
        double rightPower = 0;
        double leftPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        rightPower = -OscarMath.signumPow(axes.getRotation() * stickRotateMultiplier, 2);
        leftPower = OscarMath.signumPow(axes.getRotation() * stickRotateMultiplier, 2);

        return new TankDriveProfile(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public TankDriveProfile getTankRotateOnCenterProfile() {
        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
        double rotation = OscarMath.signumPow(axes.getRotation() * stickRotateOnCenterMultiplier, 3);

        return new TankDriveProfile(-rotation, rotation, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public ArcadeDriveProfile calculateArcadeSpeeds() {
        double xPower = 0;
        double rotPower = 0;
        DriveAxesSupplier axes = oi.driverOI.getArcadeDriveAxes();
        xPower = OscarMath.signumPow(-axes.getRight() * stickDriveMultiplier, 3);
        rotPower = OscarMath.signumPow(axes.getLeft() * stickDriveMultiplier, 3);

        return new ArcadeDriveProfile(xPower, rotPower, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public void tankDrive() {
        TankDriveProfile profile = calculateTankSpeeds();
        diffDrive.tankDrive(profile.leftPow, profile.rightPow, profile.loopMode);
//        double rightPower = 0;
//        double leftPower = 0;
//        DriveAxesSupplier axes = oi.driverOI.getTankDriveAxes();
//        double rightStick = axes.getRight();
//        double leftStick = axes.getLeft();
//        boolean isRotate = ((SticksDriverOI) oi.driverOI).rightStick.two.get();
//        boolean driveStraight = ((SticksDriverOI) oi.driverOI).leftStick.trigger.get() || ((SticksDriverOI) oi.driverOI).rightStick.trigger.get();
//
//        if (driveStraight) {
//            double power;
//            if (((SticksDriverOI) oi.driverOI).leftStick.trigger.get() && ((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
//                power = (OscarMath.signumPow(rightStick * stickDriveMultiplier, 2) + OscarMath.signumPow(leftStick * stickDriveMultiplier, 2)) / 2;
//            } else if (((SticksDriverOI) oi.driverOI).rightStick.trigger.get()) {
//                power = OscarMath.signumPow(rightStick * stickDriveMultiplier, 2);
//            } else {
//                power = (OscarMath.signumPow(leftStick * stickDriveMultiplier, 2));
//            }
//            if (isRotate) {
//                rightPower = power - OscarMath.signumPow(axes.getRotation() * stickRotateMultiplier, 2);
//                leftPower = power + OscarMath.signumPow(axes.getRotation() * stickRotateMultiplier, 2);
//            } else {
//                rightPower = power;
//                leftPower = power;
//            }
//            diffDrive.tankDrive(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
//        } else {
//            if (isRotate) {
//                double rotation = OscarMath.signumPow(axes.getRotation() * stickRotateOnCenterMultiplier, 3);
//                diffDrive.arcadeDrive(rotation, 0.0, SmartDiffDrive.LoopMode.PERCENTAGE);
//            } else {
//                rightPower = OscarMath.signumPow(rightStick * stickDriveMultiplier, 2);
//                leftPower = OscarMath.signumPow(leftStick * stickDriveMultiplier, 2);
//                diffDrive.tankDrive(leftPower, rightPower, SmartDiffDrive.LoopMode.PERCENTAGE);
//            }
//        }
    }

    public void xBoxDrive() {
        ArcadeDriveProfile profile = calculateArcadeSpeeds();
        diffDrive.arcadeDrive(profile.xPow, profile.rotPow, profile.loopMode);
    }
}
