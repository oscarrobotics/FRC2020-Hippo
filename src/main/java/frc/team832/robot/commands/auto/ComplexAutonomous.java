package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.lib.motion.PathHelper;
import frc.team832.robot.Constants;
import frc.team832.robot.Robot;
import frc.team832.robot.commands.ShootCommandGroup;
import frc.team832.robot.subsystems.*;

public class ComplexAutonomous extends SequentialCommandGroup {
	private Trajectory ToStart = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), Constants.FieldPositions.StartCenter);
	private Trajectory ToCloseSideTrench = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), Constants.FieldPositions.CloseSideTrench);
	private Trajectory ToFarSideTrench = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), Constants.FieldPositions.FarSideTrench);
	private Trajectory ToShieldGenCloseToTrench = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), Constants.FieldPositions.ShieldGenCloseToTrench);

	public ComplexAutonomous(SuperStructure superStructure, Drivetrain drivetrain, Pneumatics pneumatics, Shooter shooter, Spindexer spindexer) {
		addCommands(
			new ShootCommandGroup(superStructure, pneumatics, shooter, spindexer),
			new FollowPath(ToFarSideTrench, drivetrain),
			new InstantCommand(superStructure::intake),
			new FollowPath(ToCloseSideTrench, drivetrain),
			new InstantCommand(superStructure::idleIntake),
			new FollowPath(ToFarSideTrench, drivetrain),
			new ShootCommandGroup(superStructure, pneumatics, shooter, spindexer),
			new InstantCommand(superStructure::intake),
			new FollowPath(ToShieldGenCloseToTrench, drivetrain),
			new InstantCommand(superStructure::idleIntake),
			new FollowPath(ToFarSideTrench, drivetrain),
			new ShootCommandGroup(superStructure, pneumatics, shooter, spindexer)
			);
		addRequirements(superStructure, drivetrain, pneumatics, shooter, spindexer);
	}
}