package frc.team832.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.accesories.SpindexerStatus;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.Spindexer;

public class Shoot extends CommandBase {

    public final Shooter shooter;
    public final Spindexer spindexer;
    public double rpm;

    public Shoot(Shooter shooter, Spindexer spindexer, double rpm) {
        addRequirements(shooter, spindexer);
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        shooter.setRPM(rpm);
        spindexer.spinCounterclockwise(0.8);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
