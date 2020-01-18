package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.robot.Constants;
import frc.team832.robot.commands.TemplateCommand;

public class Climber extends SubsystemBase implements DashboardUpdatable {
    private boolean initSuccessful;
    private CANSparkMax winch;

    public Climber() {
        DashboardManager.addTab(this);
        DashboardManager.getTab(this).add(this);

        winch = new CANSparkMax(Constants.ClimberValues.WINCH_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        winch.wipeSettings();

        winch.setNeutralMode(NeutralMode.kBrake);

        winch.setInverted(false);
        winch.setSensorPhase(true);

        winch.limitInputCurrent(40);

        setDefaultCommand(new TemplateCommand(this));

        initSuccessful = true;
    }

    public boolean isInitSuccessful() { return initSuccessful; }

    @Override
    public String getDashboardTabName() {
        return "Climber";
    }

    @Override
    public void updateDashboardData() {

    }
}
