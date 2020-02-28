package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.Spindexer;
import frc.team832.robot.subsystems.Turret;


public class ShootCommandGroup extends SequentialCommandGroup {

    public ShootCommandGroup(SuperStructure superStructure) {
        addCommands(
//                new FunctionalCommand(
//                        superStructure::moveSpindexerToSafePos,
//                        () -> {},
//                        (ignored) -> superStructure.idleSpindexer(),
//                        () ->superStructure.isSpindexerReadyShoot(superStructure.getNearestSafeRotationRelativeToFeeder(), spindexer.getRelativeRotations()),
//                        spindexer),

                new InstantCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.SHOOTING)),
                new WaitCommand(3),
                new InstantCommand(() -> superStructure.setShootingState(SuperStructure.ShootingState.FIRING)),
                new WaitCommand(5),
                new InstantCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE))
        );
    }
}
