package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.Spindexer;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.Turret;

public class PrepareShooter extends CommandBase {
    private final SuperStructure superStructure;

    public PrepareShooter(SuperStructure superStructure, Shooter shooter, Spindexer spindexer, Turret turret) {
        this.superStructure = superStructure;
        addRequirements(superStructure, shooter, spindexer, turret);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructure.SuperstructureState.TARGETING);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
