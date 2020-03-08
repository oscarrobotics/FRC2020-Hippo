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
	public ComplexAutonomous(SuperStructure superStructure, Drivetrain drivetrain, Shooter shooter, Spindexer spindexer, Turret turret) {

		Trajectory ToStart = PathHelper.generatePath(drivetrain.getLatestPose(), Constants.FieldPositions.StartCenter);
		Trajectory ToCloseSideTrench = PathHelper.generatePath(drivetrain.getLatestPose(), Constants.FieldPositions.CloseSideTrench);
		Trajectory ToFarSideTrench = PathHelper.generatePath(drivetrain.getLatestPose(), Constants.FieldPositions.FarSideTrench);
		Trajectory ToShieldGenCloseToTrench = PathHelper.generatePath(drivetrain.getLatestPose(), Constants.FieldPositions.ShieldGenCloseToTrench);
		
		addCommands(
//			new ShootCommandGroup(superStructure),
//			new FollowPath(ToFarSideTrench, drivetrain),
//			new InstantCommand(superStructure::intake),
//			new FollowPath(ToCloseSideTrench, drivetrain),
//			new InstantCommand(superStructure::idleIntake),
//			new FollowPath(ToFarSideTrench, drivetrain),
//			new ShootCommandGroup(superStructure),
//			new InstantCommand(superStructure::intake),
//			new FollowPath(ToShieldGenCloseToTrench, drivetrain),
//			new InstantCommand(superStructure::idleIntake),
//			new FollowPath(ToFarSideTrench, drivetrain),
//			new ShootCommandGroup(superStructure)
		);
		addRequirements(superStructure, drivetrain, shooter, spindexer);
	}
}