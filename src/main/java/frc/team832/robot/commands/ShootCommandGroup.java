package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.Spindexer;

public class ShootCommandGroup extends SequentialCommandGroup {

    public ShootCommandGroup(SuperStructure superStructure) {
        addCommands(
                new PrepareShooter(superStructure),
                new FunctionalCommand(superStructure::shoot, () -> {} , superStructure::idleShooter, superStructure::isSpindexerUnloaded, superStructure)
        );
        addRequirements(superStructure);
    }
}
