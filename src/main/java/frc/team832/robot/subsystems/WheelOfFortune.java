package frc.team832.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverinput.controllers.StratComInterface;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.robot.Constants;
import frc.team832.robot.OI;
import frc.team832.robot.accesories.ColorWheelPath;

public class WheelOfFortune extends SubsystemBase implements DashboardUpdatable {
    private boolean initSuccessful = false;

    public CANSparkMax spinner;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
//    public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public ColorWheelPath path;

    public WheelOfFortune() {
        spinner = new CANSparkMax(Constants.WOFValues.SPINNER_CAN_ID, Motor.kNEO550);

        spinner.wipeSettings();
        spinner.setNeutralMode(NeutralMode.kBrake);

        spinner.setInverted(false);
        spinner.setSensorPhase(true);

        setCurrentLimit(40);

        initSuccessful = true;
    }

    public void senseColor() {
//        Color detectedColor = colorSensor.getColor();
//        double IR = colorSensor.getIR();
//
//        SmartDashboard.putNumber("Red", detectedColor.red);
//        SmartDashboard.putNumber("Green", detectedColor.green);
//        SmartDashboard.putNumber("Blue", detectedColor.blue);
//        SmartDashboard.putNumber("IR", IR);
//        int proximity = m_colorSensor.getProximity();
    }

    public void spinClockWise() {
        if (OI.stratComInterface.getSC2().get()) spinner.set(Constants.WOFValues.BASIC_SPIN_POW);
    }

    public void spinCounterClockWise() {
        if (OI.stratComInterface.getSC2().get()) spinner.set(-Constants.WOFValues.BASIC_SPIN_POW);
    }

    public void spinThreeRot() {
        double additionalTicks = 3 / Constants.WOFValues.RevsToTicks;
        if (OI.stratComInterface.getSC2().get()) spinner.set(spinner.getSensorPosition() + additionalTicks);
    }

    public void spinWheel(double revolutions) {
        double additionalTicks = revolutions / Constants.WOFValues.RevsToTicks;
        spinner.set(spinner.getSensorPosition() + additionalTicks);
    }

    public void setColor(ColorWheelPath.ColorWheelColor targetColor) {
//        ColorWheelPath.ColorWheelColor currentColor = path.getActualColor(colorSensor.getColor());
//        path = new ColorWheelPath(currentColor, targetColor);

        double rotations = path.rotations * (path.direction == ColorWheelPath.Direction.CLOCKWISE ? 1 : -1);//assuming that clockwise is positive
        spinner.set(rotations * Constants.WOFValues.RevsToTicks);
    }

    public void stopSpin() {
        spinner.set(0);
    }

    public void setCurrentLimit(int limit) {
        spinner.limitInputCurrent(limit);
    }

    public boolean isInitSuccessful() {
        return initSuccessful;
    }

    @Override
    public String getDashboardTabName () {
        return "Color Sensor";
    }

    @Override
    public void updateDashboardData () {

    }
}
