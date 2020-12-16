package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.team832.lib.logging.formats.FlywheelStateSpaceLogLine;
import frc.team832.lib.logging.readers.FlywheelStateSpaceLogReader;
import frc.team832.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.style.Styler;
import org.knowm.xchart.style.markers.*;

import java.util.ArrayList;
import java.util.List;

public class FlywheelStateSpaceTest {
    static final String filePath = "C:\\Users\\Banks\\Downloads\\log_14-Dec-2020_07.04.28PM_ShooterFlywheel-FlywheelStateSpace.csv";

    static List<FlywheelStateSpaceLogLine> originalDataPoints;

    @BeforeAll
    public static void init() {
        var reader = new FlywheelStateSpaceLogReader(filePath);
        originalDataPoints = reader.getDataPoints();
    }

    @Test
    public void runOrigModel() {
        var m_plant = Constants.ShooterValues.m_flywheelPlant;

        final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
                Nat.N1(), Nat.N1(),
                Constants.ShooterValues.m_flywheelPlant,
                VecBuilder.fill(2), // How accurate we think our model is
                VecBuilder.fill(0.010), // How accurate we think our encoder data is
                Constants.ShooterValues.ControlLoopPeriod);

        final LinearQuadraticRegulator<N1, N1, N1> m_controller
                = new LinearQuadraticRegulator<>(Constants.ShooterValues.m_flywheelPlant,
                VecBuilder.fill(85.0), // qelms. Velocity error tolerance, in radians per second. Decrease
                // this to more heavily penalize state excursion, or make the controller behave more aggressively.
                VecBuilder.fill(12), // relms. Control effort (voltage) tolerance. Decrease this to more
                // heavily penalize control effort, or make the controller less aggressive. 12 is a good
                // starting point because that is the (approximate) maximum voltage of a battery.
                Constants.ShooterValues.ControlLoopPeriod); // Nominal time between loops. 0.020 for TimedRobot, but can be lower if using notifiers.

        final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
                m_plant, m_controller, m_observer, 12.0, Constants.ShooterValues.ControlLoopPeriod
        );

        List<FlywheelStateSpaceLogLine> newRunDataPoints = new ArrayList<>();

        for (var dataPoint : originalDataPoints) {
            m_loop.setNextR(VecBuilder.fill(dataPoint.reference));
            m_loop.correct(VecBuilder.fill(dataPoint.output));
            m_loop.predict(Constants.ShooterValues.ControlLoopPeriod);
            newRunDataPoints.add(
                    new FlywheelStateSpaceLogLine(
                            dataPoint.fpgaTimestamp,
                            dataPoint.reference,
                            m_loop.getXHat(0),
                            m_loop.getU(0),
                            dataPoint.output
                    )
            );
        }

        final Marker[] allMarker = new Marker[] {new None(), new None(), new None(), new None()};

        XYChart origChart = new XYChartBuilder().xAxisTitle("Time").width(800).height(600).title("Original").build();

        double[] origTimestamp = originalDataPoints.stream().mapToDouble(c -> c.fpgaTimestamp).toArray();
        double[] origReference = originalDataPoints.stream().mapToDouble(c -> c.reference).toArray();
        double[] origState = originalDataPoints.stream().mapToDouble(c -> c.state).toArray();
        double[] origInput = originalDataPoints.stream().mapToDouble(c -> c.input).toArray();
        double[] origOutput = originalDataPoints.stream().mapToDouble(c -> c.output).toArray();

        origChart.getStyler().setYAxisGroupPosition(1, Styler.YAxisPosition.Right);
        origChart.getStyler().setYAxisMin(1, -13.0);
        origChart.getStyler().setYAxisMax(1, 13.0);
        origChart.setYAxisGroupTitle(0, "rad/s");
        origChart.setYAxisGroupTitle(1, "Volts");
        origChart.getStyler().setSeriesMarkers(allMarker);

        origChart.addSeries("Reference", origTimestamp, origReference).setYAxisGroup(0);
        origChart.addSeries("State", origTimestamp, origState).setYAxisGroup(0);
        origChart.addSeries("Input", origTimestamp, origInput).setYAxisGroup(1);
        origChart.addSeries("Output", origTimestamp, origOutput).setYAxisGroup(0);

        XYChart newChart = new XYChartBuilder().xAxisTitle("Time").width(800).height(600).title("New").build();

        double[] newTimestamp = newRunDataPoints.stream().mapToDouble(c -> c.fpgaTimestamp).toArray();
        double[] newReference = newRunDataPoints.stream().mapToDouble(c -> c.reference).toArray();
        double[] newState = newRunDataPoints.stream().mapToDouble(c -> c.state).toArray();
        double[] newInput = newRunDataPoints.stream().mapToDouble(c -> c.input).toArray();
        double[] newOutput = newRunDataPoints.stream().mapToDouble(c -> c.output).toArray();

        newChart.getStyler().setYAxisGroupPosition(1, Styler.YAxisPosition.Right);
        newChart.getStyler().setYAxisMin(1, -13.0);
        newChart.getStyler().setYAxisMax(1, 13.0);
        newChart.setYAxisGroupTitle(0, "rad/s");
        newChart.setYAxisGroupTitle(1, "Volts");
        origChart.setXAxisTitle("Time");
        newChart.getStyler().setSeriesMarkers(allMarker);

        newChart.addSeries("Reference", newTimestamp, newReference).setYAxisGroup(0);;
        newChart.addSeries("State", newTimestamp, newState).setYAxisGroup(0);;
        newChart.addSeries("Input", newTimestamp, newInput).setYAxisGroup(1);;
        newChart.addSeries("Output", newTimestamp, newOutput).setYAxisGroup(0);;

        new SwingWrapper<>(origChart).displayChart();
        new SwingWrapper<>(newChart).displayChart();

        try {Thread.sleep(500000);} catch(Exception ignored) {}
    }
}
