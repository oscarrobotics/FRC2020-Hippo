package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.robot.Constants;
import frc.team832.robot.accesories.ColorWheelPath;
import frc.team832.robot.commands.TemplateCommand;

public class WheelOfFortune extends SubsystemBase {
    private boolean initSuccessful = false;
    public CANSparkMax spinner;
    public ColorWheelPath path;

    public WheelOfFortune() {
        spinner = new CANSparkMax(Constants.WOFValues.SPINNER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        spinner.wipeSettings();
        spinner.setNeutralMode(NeutralMode.kBrake);

        spinner.setInverted(false);
        spinner.setSensorPhase(true);

        setCurrentLimit(40);

        setDefaultCommand(new TemplateCommand(this));

        initSuccessful = true;
    }

    public void spinWheel(double revolutions) {
        double additionalTicks = revolutions / Constants.WOFValues.RevsToTicks;
        spinner.set(spinner.getSensorPosition() + additionalTicks);
    }

    public void setColor(ColorWheelPath.ColorWheelColor targetColor) {
        ColorWheelPath.ColorWheelColor currentColor = null;//path.getActualColor(colorSensor.getColor());
        path = new ColorWheelPath(currentColor, targetColor);

        double rotations = path.rotations * (path.direction == ColorWheelPath.Direction.CLOCKWISE ? 1 : -1);//assuming that clockwise is positive
        spinner.set(rotations * Constants.WOFValues.RevsToTicks);
    }



    public void setCurrentLimit(int limit) {
        spinner.limitInputCurrent(limit);
    }

    public boolean isInitSuccessful() {
        return initSuccessful;
    }
}
