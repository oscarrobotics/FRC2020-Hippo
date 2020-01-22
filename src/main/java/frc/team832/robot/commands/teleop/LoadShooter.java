package frc.team832.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.Spindexer;

public class LoadShooter extends CommandBase {

    private final Spindexer spindexer;

    public LoadShooter(final Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.spinCounterclockwise(0.8);
        spindexer.feed(0.8);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stopSpin();
        spindexer.stopFeed();
    }
}