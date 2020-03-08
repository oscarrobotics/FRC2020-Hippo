package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.Spindexer;
import frc.team832.robot.subsystems.Turret;


public class ShootCommandGroup extends SequentialCommandGroup {

    public ShootCommandGroup(SuperStructure superStructure) {
        addCommands(
                new InstantCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.TARGETING)),
                new WaitCommand(1),
                new InstantCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.SHOOTING)),
                new WaitCommand(4),
                new InstantCommand(() -> superStructure.setState(SuperStructure.SuperstructureState.IDLE))
        );
        addRequirements(superStructure);
    }
}
