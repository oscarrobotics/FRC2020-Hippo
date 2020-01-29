package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.lib.motion.PathHelper;
import frc.team832.robot.Robot;
import frc.team832.robot.commands.ShootCommandGroup;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.Drivetrain;
import frc.team832.robot.subsystems.Spindexer;

public class BasicAutonomous extends SequentialCommandGroup {

    private Trajectory BackUp = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), new Pose2d(-1.0, 0, Rotation2d.fromDegrees(0))); //TODO: Change coordinates

    public BasicAutonomous(SuperStructure superStructure, Spindexer spindexer, Drivetrain drivetrain) {
        addCommands(
                new ShootCommandGroup(superStructure),
                new FollowPath(BackUp, drivetrain)
        );
        addRequirements(superStructure, spindexer, drivetrain);
    }
}
