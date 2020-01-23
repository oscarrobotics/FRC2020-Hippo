package frc.team832.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants;
import frc.team832.robot.accesories.SpindexerStatus;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.Spindexer;

public class Shoot extends CommandBase {

    public final Shooter shooter;
    public final Spindexer spindexer;

    public Shoot(Shooter shooter, Spindexer spindexer) {
        addRequirements(shooter, spindexer);
        this.shooter = shooter;
        this.spindexer = spindexer;
    }

    @Override
    public void initialize() {
        shooter.setMode(Shooter.ShootMode.Shooting);
        spindexer.setFeedRPM(1200);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setMode(Shooter.ShootMode.Idle);
    }
}
