package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.Intake;
import frc.team832.robot.subsystems.Spindexer;

public class IntakePowerCell extends CommandBase {
    private final Intake intake;
    private final Spindexer spindexer;

    public IntakePowerCell(final Intake intake, Spindexer spindexer) {
        this.intake = intake;
        this.spindexer = spindexer;
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize() {
        intake.intake(0.8);
        spindexer.spinClockwise(0.8);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        spindexer.stopSpin();
    }
}