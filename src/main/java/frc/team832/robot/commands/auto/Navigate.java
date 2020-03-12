package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.lib.motion.PathHelper;
import frc.team832.robot.Constants;
import frc.team832.robot.Constants.DrivetrainValues;
import frc.team832.robot.subsystems.Drivetrain;

public class Navigate extends SequentialCommandGroup {
    private static final RamseteController ramseteController = new RamseteController();
    private static final PIDController leftDrivePIDController = new PIDController(DrivetrainValues.LeftkP, 0, 0);
    private static final PIDController rightDrivePIDController = new PIDController(DrivetrainValues.RightkP, 0, 0);

    public Navigate(Pose2d endPose, Drivetrain drivetrain) {
        Trajectory path = PathHelper.generatePath(drivetrain.getLatestPose(), endPose, Constants.DrivetrainValues.LeftTrajectoryConfig);
        addCommands(
        new FollowPath(path, drivetrain)
        );
    }
}