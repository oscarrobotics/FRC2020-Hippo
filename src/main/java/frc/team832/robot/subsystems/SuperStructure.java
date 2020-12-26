package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class SuperStructure extends SubsystemBase {

    private final Intake intake;
    private final Shooter shooter;
    private final Spindexer spindexer;
    private final Turret turret;
    private final Vision vision;

    private final NetworkTableEntry dashboard_hoodVolts;

    public final IdleCommand idleCommand;
    public final TargetingCommand targetingCommand;
    public final ShootCommandGroup shootOnTarget;
    public final CloseRangeShootCommandGroup closeShoot;
    public final TargetingTestCommand testTargeting;
    public final ExtendIntakeCommand extendIntake;
    public final ExtendOuttakeCommand extendOuttake;
    public final RetractIntakeCommand retractIntake;


    public SuperStructure(Intake intake, Shooter shooter, Spindexer spindexer, Turret turret, Vision vision) {
        this.intake = intake;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.turret = turret;
        this.vision = vision;

        idleCommand = new IdleCommand();
        targetingCommand = new TargetingCommand();
        shootOnTarget = new ShootCommandGroup();
        closeShoot = new CloseRangeShootCommandGroup();
        testTargeting = new TargetingTestCommand();
        extendIntake = new ExtendIntakeCommand();
        extendOuttake = new ExtendOuttakeCommand();
        retractIntake = new RetractIntakeCommand();

        DashboardManager.addTab(this);
        dashboard_hoodVolts = DashboardManager.addTabItem(this, "Current Volts", 0.0);

        vision.driverMode(false);
    }

    @Override
    public void periodic() {
        dashboard_hoodVolts.setDouble(shooter.getPotentiometer());
    }

    public void trackTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget(spindexer.getRPM());
            shooter.trackTarget();
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }

    public void testTrackTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget(spindexer.getRPM());
            shooter.setHoodToVisionDistance();
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }

    public void shootAtTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget(spindexer.getRPM());
            shooter.trackTarget();
            shooter.setFeedRPM(Constants.ShooterValues.FeedRpm);
            spindexer.setSpinRPM(ShooterCalculations.getSpindexerRpm(), Spindexer.SpinnerDirection.Clockwise);
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }

    public boolean readyToShoot() {
        return shooter.atShootingRpm() && shooter.atHoodAngle() && turret.atTargetAngle();
    }


    private class IdleCommand extends InstantCommand {
        IdleCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            intake.stopAll();
            shooter.idleAll();
            spindexer.idle();
            turret.setForward();
        }
    }

    public class TargetingCommand extends CommandBase {
        TargetingCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            spindexer.idle();
            turret.setForward();
        }

        @Override
        public void execute() {
            trackTarget();
        }
    }

    public class ShootCommandGroup extends ParallelCommandGroup {
        public ShootCommandGroup() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
            addCommands(
                    new RunEndCommand(
                            SuperStructure.this::trackTarget,
                            () -> {
                                shooter.idleAll();
                                spindexer.idle();
                            }
                    ),
                    new SequentialCommandGroup(
                            new WaitUntilCommand(SuperStructure.this::readyToShoot),
                            new InstantCommand(() -> shooter.setFeedRPM(3000)),
                            new WaitUntilCommand(shooter::atFeedRpm),
                            new RunEndCommand(
                                    SuperStructure.this::shootAtTarget,
                                    () -> {
                                        shooter.idleAll();
                                        spindexer.idle();
                                    }
                            )
                    )
            );
        }
    }

    /**
     * on start:
     * setFlywheel(5000)
     * setHood(16)
     * setTurretStraight()
     * <p>
     * waitUntil(allAboveReady)
     * setFeeder(3000)
     * waitUntil(feederReady)
     * setSpindexer(120 clockwise)
     * <p>
     * on end:
     * stopAll
     */

    public class CloseRangeShootCommandGroup extends ParallelCommandGroup {
        public CloseRangeShootCommandGroup() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
            addCommands(
                new StartEndCommand(
                    () -> {
                        shooter.setFlywheelRPM(4500);
                        shooter.setHoodAngle(19);
                        turret.setForward();
                    },
                    () -> {
                        shooter.idleAll();
                        spindexer.idle();
                    }
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooter.atShootingRpm() && shooter.atHoodAngle() && turret.atTargetAngle()),
                        new InstantCommand(() -> shooter.setFeedRPM(2500)),
                        new WaitUntilCommand(shooter::atFeedRpm),
                        new StartEndCommand(
                                () -> spindexer.setSpinRPM(100, Spindexer.SpinnerDirection.Clockwise),
                                () -> {
                                    shooter.idleAll();
                                    spindexer.idle();
                                }
                        )
                )
            );
        }
    }

    public class ExtendIntakeCommand extends SequentialCommandGroup {
        public ExtendIntakeCommand() {
            addRequirements(intake, shooter, spindexer, turret, SuperStructure.this);
            addCommands(
                    new InstantCommand(intake::extendIntake),
                    new InstantCommand(() -> spindexer.setSpinRPM(30, Spindexer.SpinnerDirection.Clockwise)),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> intake.intake(0.5)),
                    spindexer.getAntiJamSpinCommand(10, 1.0)
            );
        }
    }

    public class ExtendOuttakeCommand extends SequentialCommandGroup {
        public ExtendOuttakeCommand() {
            addRequirements(intake, shooter, spindexer, turret, SuperStructure.this);
            addCommands(
                    new InstantCommand(intake::extendIntake),
                    new InstantCommand(() -> spindexer.setSpinRPM(30, Spindexer.SpinnerDirection.CounterClockwise)),
                    new WaitCommand(0.25),
                    new InstantCommand(() -> intake.outtake(0.4))
            );
        }
    }

    public class RetractIntakeCommand extends SequentialCommandGroup {
        public RetractIntakeCommand() {
            addRequirements(intake, shooter, spindexer, turret, SuperStructure.this);
            addCommands(
                    new InstantCommand(intake::retractIntake),
                    new WaitCommand(1.0),
                    new InstantCommand(intake::stop),
                    new InstantCommand(spindexer::idle)
            );
        }
    }

    public class TargetingTestCommand extends CommandBase {
        TargetingTestCommand() {
            addRequirements(shooter, intake, spindexer, turret, SuperStructure.this);
        }

        @Override
        public void initialize() {
            spindexer.idle();
            turret.setForward();
            spindexer.setSpinRPM(60, Spindexer.SpinnerDirection.Clockwise);
        }

        @Override
        public void execute() {
            testTrackTarget();
        }

        @Override
        public void end(boolean interrupted) {
            shooter.idleAll();
            spindexer.idle();
        }
    }
}
