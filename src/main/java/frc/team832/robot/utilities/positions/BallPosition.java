package frc.team832.robot.utilities.positions;

public enum BallPosition {
	Position0(0.1, 0),
	Position1(0.3, 1),
	Position2(0.5, 2),
	Position3(0.7, 3),
	Position4(0.9, 4);

	public final double rotations;
	public final int slotNumber;

	BallPosition(double rotations, int slotNum) {
		this.rotations = rotations;
		this.slotNumber = slotNum;
	}
}
