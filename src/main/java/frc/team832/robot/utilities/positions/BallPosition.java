package frc.team832.robot.utilities.positions;

public enum BallPosition {
	Position1(0.0, 0),
	Position2(0.2, 1),
	Position3(0.4, 2),
	Position4(0.6, 3),
	Position5(0.8, 4);

	public final double rotations;
	public final int slotNumber;

	BallPosition(double rotations, int slotNum) {
		this.rotations = rotations;
		this.slotNumber = slotNum;
	}
}
