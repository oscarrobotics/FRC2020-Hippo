package frc.team832.robot.utilities.positions;

public enum BallPosition {
	Position1(0.0),
	Position2(0.2),
	Position3(0.4),
	Position4(0.6),
	Position5(0.8);

	public final double value;

	BallPosition(double value) {
		this.value = value;
	}
}
