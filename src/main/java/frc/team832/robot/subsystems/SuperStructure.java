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
            turret.trackTarget(spindexer.getVelocity());
            shooter.trackTarget();
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }

    public void testTrackTarget() {
        if (vision.hasTarget()) {
            turret.trackTarget(spindexer.getVelocity());
            shooter.setHoodToVisionDistance();
        } else {
            turret.setTurretTargetDegrees(0.0, true);
        }
    }

    public void shootAtTarget() {
        if (vision.hasTarget()) {
            turret.holdPosition();
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
            intake.idle();
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
                    new SequentialCommandGroup(
                            new FunctionalCommand(() -> {}, SuperStructure.this::trackTarget, (interrupted) -> {turret.holdPosition();},
                                    SuperStructure.this::readyToShoot),
                            new InstantCommand(() -> shooter.setFeedRPM(3000)),
                            new WaitUntilCommand(shooter::atFeedRpm),
                            new FunctionalCommand(() -> {}, SuperStructure.this::trackTarget, (interrupted) -> {turret.setLastYaw();},
                                    SuperStructure.this::readyToShoot),
                            new RunEndCommand(
                                    SuperStructure.this::shootAtTarget,
                                    () -> {
                                        shooter.setFlywheelRPM(0);
                                        shooter.setFeedRPM(0);
                                        shooter.setHoodAngle(45);
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
