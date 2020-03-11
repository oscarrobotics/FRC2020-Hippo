package frc.team832.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.robot.subsystems.Climber;

public class StartClimbGroup extends SequentialCommandGroup {

    public StartClimbGroup(Climber climber, boolean climbUp){
        addCommands(
                new InstantCommand(climber::unlockClimb, climber),
                new WaitCommand(0.2),
                new InstantCommand(climbUp ? climber::windWinch : climber::unwindWinch, climber)
        );
    }
}
