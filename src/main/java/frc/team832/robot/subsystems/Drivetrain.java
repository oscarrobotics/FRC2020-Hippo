package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.OI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.robot.Constants;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.robot.commands.TemplateCommand;

public class Drivetrain extends SubsystemBase {
    private boolean initSuccessful = false;
    private CANTalonFX rightMaster, rightSlave, leftMaster, leftSlave;
    private SmartDiffDrive diffDrive;

    public Drivetrain() {
        leftMaster = new CANTalonFX(Constants.LEFT_MASTER_CAN_ID);
        leftSlave = new CANTalonFX(Constants.LEFT_SLAVE_CAN_ID);
        rightMaster = new CANTalonFX(Constants.RIGHT_MASTER_CAN_ID);
        rightSlave = new CANTalonFX(Constants.RIGHT_SLAVE_CAN_ID);

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

        diffDrive = new SmartDiffDrive(leftMaster, rightMaster, Constants.MAX_RPM);

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
        double moveStick = -OI.drivePad.getY(GenericHID.Hand.kLeft);
        double rotStick = OI.drivePad.getX(GenericHID.Hand.kRight);

        boolean preciseRot = OI.drivePad.rightBumper.get();
        boolean preciseMove = OI.drivePad.leftBumper.get();

//        boolean visionRot = OI.drivePad.rightStickPress.get();

        moveStick = OscarMath.signumPow(moveStick, 2);
        rotStick = OscarMath.signumPow(rotStick, 4);

        double rotPow = preciseRot ? rotStick * Constants.PRECISE_ROT_MULTIPLIER : rotStick * Constants.DEFAULT_ROT_MULTIPLIER;
        double movePow = preciseMove ? moveStick * Constants.PRECISE_MOVE_MULTIPLIER : moveStick * Constants.DEFAULT_MOVE_MULTIPLIER;

//        OI.vision.setDriverMode(!visionRot);
//        if (visionRot) {
//            rotPow = OI.vision.getYaw() * OI.vision.getYawKp();
//        }

        diffDrive.curvatureDrive(movePow, rotPow, true, SmartDiffDrive.LoopMode.PERCENTAGE);
    }

    public void tankDrive() {
        double leftStick = Math.abs(OI.leftDriveStick.getY()) > 0.025 ? OI.leftDriveStick.getY() : 0;
        double rightStick =  Math.abs(OI.rightDriveStick.getY()) > 0.025 ? OI.rightDriveStick.getY() : 0;
        double leftPow, leftAdjusted;
        double rightPow, rightAdjusted;
        boolean isTwistTurn = false;

        if (Math.abs(leftStick - rightStick) < 0.075 && (Math.abs(leftStick) > 0.1 && Math.abs(rightStick) > 0.1)) {
            leftAdjusted = (leftStick + rightStick) / 2;
            rightAdjusted = (leftStick + rightStick) / 2;
        } else if (Math.abs(leftStick) < 0.1 && Math.abs(rightStick) < 0.1) {
            leftAdjusted = 0;
            rightAdjusted = 0;
            isTwistTurn = true;
            double rot = Math.pow(OI.rightDriveStick.getTwist(), 2.1);//.1 is to preserve sign
            diffDrive.curvatureDrive(0, rot, true, SmartDiffDrive.LoopMode.PERCENTAGE);
        } else {
            leftAdjusted = leftStick;
            rightAdjusted = rightStick;
        }

        leftPow = Math.pow(leftAdjusted, 1.5) * (OI.leftDriveStick.getTrigger() ? Constants.PRECISE_TANK_MULTIPLIER : Constants.DEFAULT_TANK_MULTIPLIER);
        rightPow = Math.pow(rightAdjusted, 1.5) * (OI.rightDriveStick.getTrigger() ? Constants.PRECISE_TANK_MULTIPLIER : Constants.DEFAULT_TANK_MULTIPLIER);

        if (!isTwistTurn) {
            rightMaster.set(rightPow);
            leftMaster.set(-leftPow);
        }
    }
}