package frc.team832.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.Intake;
import frc.team832.robot.subsystems.Spindexer;

public class ManipulatePowerCell extends CommandBase {
    private final Intake intake;
    private final Spindexer spindexer;

    public enum IntakeState {
        Intake,
        Outake
    }

    private final IntakeState intakeState;

    public ManipulatePowerCell(final Intake intake, final Spindexer spindexer, IntakeState intakeState) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.intakeState = intakeState;
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize() {
        if (intakeState == IntakeState.Intake) {
            intake.intake(0.8);
            spindexer.spinClockwise(0.8);
        } else intake.outtake(0.8);
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