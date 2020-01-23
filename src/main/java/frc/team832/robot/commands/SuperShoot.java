package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.SuperStructure;
import frc.team832.robot.subsystems.Spindexer;

public class SuperShoot extends CommandBase {

    private final SuperStructure superStructure;
    private final Spindexer spindexer;
    private int rotCount = 0;

    public SuperShoot (SuperStructure superStructure, Spindexer spindexer) {
        this.superStructure = superStructure;
        this.spindexer = spindexer;
        addRequirements(superStructure, spindexer);
    }

    @Override
    public void initialize() {
        superStructure.setMode(SuperStructure.SuperstructureMode.Shooting);

    }

    @Override
    public void execute() {
        if(spindexer.getHallEffect()){
            rotCount++;
        }
    }

    @Override
    public boolean isFinished() {
        return rotCount >= 2;
    }

    @Override
    public void end(boolean interrupted) {
        superStructure.setMode(SuperStructure.SuperstructureMode.Idle);
    }
}
