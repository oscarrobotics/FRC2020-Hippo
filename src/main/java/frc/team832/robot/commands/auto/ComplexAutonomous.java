package frc.team832.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.lib.motion.PathHelper;
import frc.team832.robot.Constants;
import frc.team832.robot.Robot;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.Drivetrain;
import frc.team832.robot.subsystems.Spindexer;

public class ComplexAutonomous extends SequentialCommandGroup {
	private Trajectory ToStart = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), Constants.FieldPositions.StartCenter);
	private Trajectory ToCloseSideTrench = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), Constants.FieldPositions.CloseSideTrench);
	private Trajectory ToFarSideTrench = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), Constants.FieldPositions.FarSideTrench);
	private Trajectory ToShieldGenCloseToTrench = PathHelper.generatePath(Robot.drivetrain.getLatestPose(), Constants.FieldPositions.ShieldGenCloseToTrench);

	public ComplexAutonomous(SuperStructure superStructure, Spindexer spindexer, Drivetrain drivetrain) {
		addCommands(
			new ShootCommandGroup(superStructure, spindexer),
			new FollowPath(ToFarSideTrench, drivetrain),
			new InstantCommand(() -> Robot.superStructure.setMode(SuperStructure.SuperstructureMode.Intake)),
			new FollowPath(ToCloseSideTrench, drivetrain),
			new InstantCommand(() -> Robot.superStructure.setMode(SuperStructure.SuperstructureMode.Idle)),
			new FollowPath(ToFarSideTrench, drivetrain),
			new ShootCommandGroup(superStructure, spindexer),
			new InstantCommand(() -> Robot.superStructure.setMode(SuperStructure.SuperstructureMode.Intake)),
			new FollowPath(ToShieldGenCloseToTrench, drivetrain),
			new InstantCommand(() -> Robot.superStructure.setMode(SuperStructure.SuperstructureMode.Idle)),
			new FollowPath(ToFarSideTrench, drivetrain),
			new ShootCommandGroup(superStructure, spindexer)
			);
		addRequirements(superStructure, spindexer, drivetrain);


//		addCommands(
//			new ShootCommandGroup(superStructure, spindexer),
//			new FollowPath(ToFarSideTrench, drivetrain),
//			new InstantCommand(() -> Robot.superStructure.setMode(SuperStructure.SuperstructureMode.Intake)),
//			new FollowPath(ToCloseSideTrench, drivetrain),
//			new InstantCommand(() -> Robot.superStructure.setMode(SuperStructure.SuperstructureMode.Idle)),
//			new FollowPath(ToFarSideTrench, drivetrain),
//			new ShootCommandGroup(superStructure, spindexer)
//		);
//		addRequirements(superStructure, spindexer, drivetrain);
	}
}