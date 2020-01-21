package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.lib.motion.PathHelper;
import frc.team832.robot.subsystems.Drivetrain;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.Spindexer;

]
public class BasicAutonomous extends SequentialCommandGroup {

    public BasicAutonomous(Drivetrain drivetrain, Shooter shooter, Spindexer spindexer) {
        addCommands(
                new Shoot(shooter, spindexer, 4000),
                new RamseteCommand();

        );
    }
}
