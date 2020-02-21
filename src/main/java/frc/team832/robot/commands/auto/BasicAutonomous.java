package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.lib.motion.PathHelper;
import frc.team832.robot.Robot;
import frc.team832.robot.commands.ShootCommandGroup;
import frc.team832.robot.subsystems.*;

public class BasicAutonomous extends SequentialCommandGroup {
    public BasicAutonomous(SuperStructure superStructure, Drivetrain drivetrain, Shooter shooter, Spindexer spindexer, Intake intake) {
        var path = PathHelper.generatePath(drivetrain.getLatestPose(), new Pose2d(-1.0, 0, Rotation2d.fromDegrees(0))); //TODO: Change coordinates
        addCommands(
                new ShootCommandGroup(superStructure, shooter, spindexer),
                new FollowPath(path, drivetrain)
        );
        addRequirements(superStructure, drivetrain, shooter, spindexer, intake);
    }
}
