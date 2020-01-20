package frc.team832.robot.accesories;


public class ShooterCalculations {
    public double flywheelRPM, hoodAngle;

    public void calculate(double distanceMeters) {
        hoodAngle = Math.tan(2.5 / distanceMeters);
        flywheelRPM = (distanceMeters * 300) + 5000;
    }
}
