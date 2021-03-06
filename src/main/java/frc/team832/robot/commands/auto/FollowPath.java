package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.OscarRamseteCommand;
import frc.team832.robot.Constants.DrivetrainValues;
import frc.team832.robot.subsystems.Drivetrain;

public class FollowPath extends OscarRamseteCommand {
    private static final RamseteController ramseteController = new RamseteController();
    private static final PIDController leftDrivePIDController = new PIDController(DrivetrainValues.LeftkP, 0, 0);
    private static final PIDController rightDrivePIDController = new PIDController(DrivetrainValues.RightkP, 0, 0);

    public FollowPath(Trajectory trajectory, Drivetrain drivetrain) {
        super(trajectory, drivetrain::getLatestPose, ramseteController, DrivetrainValues.LeftFF,
                DrivetrainValues.RightFF, DrivetrainValues.DriveKinematics,
                drivetrain::getWheelSpeeds, leftDrivePIDController, rightDrivePIDController,
                drivetrain::setWheelVolts, drivetrain);
        addRequirements(drivetrain);
    }
}
