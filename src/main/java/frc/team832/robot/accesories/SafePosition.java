package frc.team832.robot.accesories;

public enum SafePosition {
	Position1(0.1),
	Position2(0.3),
	Position3(0.5),
	Position4(0.7),
	Position5(0.9);

	public final double value;
	SafePosition(double value) {
		this.value = value;
	}
}
