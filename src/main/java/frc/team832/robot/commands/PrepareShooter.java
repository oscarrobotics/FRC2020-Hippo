package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.SuperStructure;

public class PrepareShooter extends CommandBase {
    private final SuperStructure superStructure;

    public PrepareShooter(SuperStructure superStructure) {
        this.superStructure = superStructure;
    }

    @Override
    public void initialize() {
        superStructure.prepareShoot();
    }

    @Override
    public boolean isFinished() {
        return superStructure.isShooterPrepared();
    }
}
