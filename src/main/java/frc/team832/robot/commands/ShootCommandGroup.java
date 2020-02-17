package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.Robot;
import frc.team832.robot.subsystems.Pneumatics;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.Spindexer;

public class ShootCommandGroup extends SequentialCommandGroup {

    public ShootCommandGroup(SuperStructure superStructure, Pneumatics pneumatics, Shooter shooter, Spindexer spindexer) {
        addCommands(
                new FunctionalCommand(
                        () -> spindexer.setTargetPosition(spindexer.getNearestSafeRotationRelativeToFeeder()),
                        () -> {},
                        (ignored) -> spindexer.stopSpin(),
                        spindexer::isSafe,
                        Robot.spindexer),

                new PrepareShooter(superStructure, pneumatics, shooter, spindexer),

                new FunctionalCommand(
                        superStructure::shoot,
                        () -> {},
                        (ignored) -> superStructure.idleShooter(),
                        superStructure::isSpindexerUnloaded,
                        superStructure)
        );
        addRequirements(superStructure, pneumatics, shooter, spindexer);
    }
}
