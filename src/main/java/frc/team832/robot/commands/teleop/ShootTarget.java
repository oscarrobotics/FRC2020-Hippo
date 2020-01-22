package frc.team832.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.accesories.ShooterCalculations;
import frc.team832.robot.subsystems.Shooter;
import frc.team832.robot.subsystems.Spindexer;

public class ShootTarget extends CommandBase {
    private final Shooter shooter;
    private ShooterCalculations calculations = new ShooterCalculations();

    public ShootTarget(final Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterMode(Shooter.SHOOTER_MODE.SHOOTING);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMode(Shooter.SHOOTER_MODE.IDLE);
    }
}
