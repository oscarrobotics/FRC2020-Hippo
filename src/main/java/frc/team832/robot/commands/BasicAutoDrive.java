package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team832.robot.subsystems.Drivetrain;

public class BasicAutoDrive extends RamseteCommand {

    private final Drivetrain drivetrain;

    public BasicAutoDrive(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.driveOdometry.
    }
}
