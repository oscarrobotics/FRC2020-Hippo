package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardUpdatable;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.robot.Constants;
import frc.team832.robot.OI;
import frc.team832.robot.utilities.positions.ColorWheelPath;

import java.awt.dnd.DropTarget;

public class WheelOfFortune extends SubsystemBase implements DashboardUpdatable {
    public final boolean initSuccessful;
    private final CANSparkMax spinner;

    private Solenoid wheelOfFortunePneumatics;

//    public CANSparkMax spinner;

    private ProfiledPIDController pid = new ProfiledPIDController(Constants.WOFValues.kP, 0, 0, Constants.WOFValues.Constraints);

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
//    public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    private ColorWheelPath path;

    public WheelOfFortune() {
        spinner = new CANSparkMax(Constants.WOFValues.SPINNER_CAN_ID, Motor.kNEO550);
        wheelOfFortunePneumatics = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.WHEEL_O_FORTUNE_SOLENOID_ID);

        spinner.wipeSettings();
        spinner.setNeutralMode(NeutralMode.kBrake);

        spinner.setInverted(false);
        spinner.setSensorPhase(true);

        setCurrentLimit(40);

        initSuccessful = spinner.getCANConnection();
    }

//    public void senseColor() {
//        Color detectedColor = colorSensor.getColor();
//        double IR = colorSensor.getIR();
//
//        SmartDashboard.putNumber("Red", detectedColor.red);
//        SmartDashboard.putNumber("Green", detectedColor.green);
//        SmartDashboard.putNumber("Blue", detectedColor.blue);
//        SmartDashboard.putNumber("IR", IR);
//        int proximity = m_colorSensor.getProximity();
//    }

    public void spinClockwise() {
        spinner.set(0.5);
    }

    public void spinCounterclockwise() {
        spinner.set(-0.5);
    }

    public void spinThreeTimes() {
        spinWheel(3);
    }

    public void spinWheel(double rotations) {
        double currentRotations = (spinner.getSensorPosition() / Constants.WOFValues.SpinReduction) % 1;
        double targetRotations = currentRotations + rotations;
        spinner.set(pid.calculate(currentRotations, targetRotations));
    }

    public void setColor(ColorWheelPath.ColorWheelColor targetColor) {
//        ColorWheelPath.ColorWheelColor currentColor = path.getActualColor(colorSensor.getColor());
//        path = new ColorWheelPath(currentColor, targetColor);

        double rotations = path.rotations * (path.direction == ColorWheelPath.Direction.CLOCKWISE ? 1 : -1);//assuming that clockwise is positive
        spinWheel(rotations);
    }

    public void stopSpin() {
        spinner.set(0);
    }

    public void extendWOFManipulator() {
        wheelOfFortunePneumatics.set(true);
    }

    public void retractWOFManipulator() {
        wheelOfFortunePneumatics.set(true);
    }

    public void setCurrentLimit(int limit) {
        spinner.limitInputCurrent(limit);
    }

    @Override
    public String getDashboardTabName () {
        return "Color Sensor";
    }

    @Override
    public void updateDashboardData () {

    }
}
