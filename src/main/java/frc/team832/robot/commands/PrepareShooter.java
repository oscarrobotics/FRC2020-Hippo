package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.Pneumatics;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.Spindexer;
import frc.team832.robot.subsystems.SuperStructure;

public class PrepareShooter extends CommandBase {
    private final SuperStructure superStructure;

    public PrepareShooter(SuperStructure superStructure, Pneumatics pneumatics, Shooter shooter, Spindexer spindexer) {
        this.superStructure = superStructure;
        addRequirements(superStructure, pneumatics, shooter, spindexer);
    }

    @Override
    public void initialize() {
        superStructure.setMode(SuperStructure.SuperStructureMode.SPINUP);
    }

    @Override
    public boolean isFinished() {
        return superStructure.isShooterPrepared();
    }
}
