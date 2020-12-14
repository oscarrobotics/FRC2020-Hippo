package frc.team832.robot.subsystems;

import frc.team832.lib.util.OscarMath;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonUtils;

import java.util.Arrays;
import java.util.stream.Collectors;

public class DistanceTest {

    final double[] pitchValues = {18.12, 9.2, 2.19, -1.57, -5.27, -8.19, -11.14};
    final double[] actualMeters = {1.2954, 1.752, 2.286, 2.5908, 2.9718, 3.2512, 3.6576};
    final double cameraAngleTwo = 90 - 33.47; // 33.47
    final double cameraHeightMeters = 0.43;
    final double powerPortHeightMeters = 2.11;

    @Test
    public void checkDistance() {
        double[] reportedMetersTwo = new double[pitchValues.length];
        for (int i = 0; i < pitchValues.length; i++) {
            reportedMetersTwo[i] = PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, powerPortHeightMeters, Math.toRadians(cameraAngleTwo), Math.toRadians(pitchValues[i]));
            reportedMetersTwo[i] *= (1 / 0.45);
            reportedMetersTwo[i] = OscarMath.round(reportedMetersTwo[i], 4);
        }
        var actual = Arrays.stream(actualMeters).mapToObj(String::valueOf).collect(Collectors.joining(", "));
        var reported = Arrays.stream(reportedMetersTwo).mapToObj(String::valueOf).collect(Collectors.joining(", "));
        System.out.println("Actual: " + actual);
        System.out.println("Reported: " + reported);
    }

}
