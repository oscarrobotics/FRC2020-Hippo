package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.Spindexer;

public class ShootCommandGroup extends SequentialCommandGroup {

    public ShootCommandGroup(SuperStructure superStructure, Shooter shooter, Spindexer spindexer) {
        addCommands(
                new FunctionalCommand(
                        () -> spindexer.setTargetPosition(spindexer.getNearestSafeRotationRelativeToFeeder()),
                        () -> {},
                        (ignored) -> spindexer.stopSpin(),
                        spindexer::isSafe,
                        spindexer),

                new PrepareShooter(superStructure, shooter, spindexer),

                new FunctionalCommand(
                        superStructure::shoot,
                        () -> {},
                        (ignored) -> superStructure.idleShooter(),
                        superStructure::isSpindexerUnloaded,
                        superStructure)
        );
        addRequirements(superStructure, shooter, spindexer);
    }
}
