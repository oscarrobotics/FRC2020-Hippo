package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.SuperStructure;
import frc.team832.robot.commands.teleop.PrepareShooter;
import frc.team832.robot.subsystems.Spindexer;

public class ShootCommandGroup extends SequentialCommandGroup {

    public ShootCommandGroup(SuperStructure superStructure, Spindexer spindexer) {
        addCommands(
            new PrepareShooter(superStructure),
            new SuperShoot(superStructure, spindexer)
        );
        addRequirements(superStructure, spindexer);
    }
}
