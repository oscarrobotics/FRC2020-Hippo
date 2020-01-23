package frc.team832.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.SuperStructure;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.Spindexer;

import static frc.team832.robot.Robot.shooter;


public class PrepareShooter extends CommandBase {
    private final SuperStructure superStructure;

    public PrepareShooter(SuperStructure superStructure) {
        this.superStructure = superStructure;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setMode(SuperStructure.SuperstructureMode.PrepareShoot);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return superStructure.isShooterPrepared();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setMode(Shooter.ShootMode.Shooting);
    }
}

