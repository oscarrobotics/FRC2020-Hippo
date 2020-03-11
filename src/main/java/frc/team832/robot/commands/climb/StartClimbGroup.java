package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.robot.subsystems.Climber;

public class ClimbGroup extends SequentialCommandGroup {

    public ClimbGroup(Climber climber, boolean climbUp){
        addCommands(
                new InstantCommand(climber::unlockClimb, climber),
                new WaitCommand(0.2),
                new StartEndCommand(climbUp ? climber::windWinch : climber::unwindWinch, climber::stopClimb)
        );
    }
}
