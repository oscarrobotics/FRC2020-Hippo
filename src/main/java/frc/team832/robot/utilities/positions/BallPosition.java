package frc.team832.robot.utilities.positions;

public enum BallPosition {
	Position1(0.0, 1),
	Position2(0.2, 2),
	Position3(0.4, 3),
	Position4(0.6, 4),
	Position5(0.8, 5);

	public final double rotations;
	public final int slotNumber;

	BallPosition(double rotations, int slotNum) {
		this.rotations = rotations;
		this.slotNumber = slotNum;
	}
}
