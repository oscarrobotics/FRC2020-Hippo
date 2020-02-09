package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team832.robot.Constants;
import frc.team832.robot.Robot;
import frc.team832.robot.subsystems.Drivetrain;

public class FollowPath extends RamseteCommand {
    private static final RamseteController ramseteController = new RamseteController();
    private static final PIDController leftDrivePIDController = new PIDController(Constants.DrivetrainValues.LeftkP, 0, 0);
    private static final PIDController rightDrivePIDController = new PIDController(Constants.DrivetrainValues.RightkP, 0, 0);

    public FollowPath (Trajectory trajectory, Drivetrain drivetrain) {
        super(trajectory, Robot.drivetrain::getLatestPose, ramseteController, Constants.DrivetrainValues.kDriveFF, Constants.DrivetrainValues.kDriveFF, Constants.DrivetrainValues.DriveKinematics, Robot.drivetrain::getWheelSpeeds, leftDrivePIDController, rightDrivePIDController, Robot.drivetrain::setWheelVolts, Robot.drivetrain);
        addRequirements(drivetrain);
    }
}
