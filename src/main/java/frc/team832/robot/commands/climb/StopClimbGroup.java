package frc.team832.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.robot.subsystems.Climber;

public class StopClimbGroup extends SequentialCommandGroup {

    public StopClimbGroup(Climber climber){
        addCommands(
                new InstantCommand(climber::stopClimb),
                new WaitCommand(0.2),
                new InstantCommand(climber::lockClimb)
        );
    }
}
