package frc.team832.robot.utilities.positions;

public class ColorWheelPath {
   public final Direction direction;
   public final double rotations;

    public enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    public enum ColorWheelColor {
        RED,
        GREEN,
        BLUE,
        YELLOW
    }

    public ColorWheelColor getActualColor(ColorWheelColor currentColor) {
        switch (currentColor) {
            case RED:
                return ColorWheelColor.BLUE;
            case GREEN:
                return ColorWheelColor.YELLOW;
            case BLUE:
                return ColorWheelColor.RED;
            case YELLOW:
                return ColorWheelColor.GREEN;
            default:
                return currentColor;
        }
    }

    public ColorWheelPath(ColorWheelColor initial, ColorWheelColor end) {
        switch(end) {
            case RED:
                switch (initial) {
                    case BLUE:
                        direction = Direction.CLOCKWISE;
                        rotations = 0.25;
                        break;
                    case GREEN:
                        direction = Direction.COUNTERCLOCKWISE;
                        rotations = 0.125;
                        break;
                    case YELLOW:
                        direction = Direction.CLOCKWISE;
                        rotations = 0.125;
                        break;
                    default:
                        direction = Direction.CLOCKWISE;
                        rotations = 0;
                }
            break;
            case BLUE:
                switch (initial) {
                    case RED:
                        direction = Direction.CLOCKWISE;
                        rotations = 0.25;
                        break;
                    case GREEN:
                        direction = Direction.CLOCKWISE;
                        rotations = 0.125;
                        break;
                    case YELLOW:
                        direction = Direction.COUNTERCLOCKWISE;
                        rotations = 0.125;
                        break;
                    default:
                        direction = Direction.CLOCKWISE;
                        rotations = 0;
                }
            break;
            case GREEN:
                switch (initial) {
                    case BLUE:
                        direction = Direction.COUNTERCLOCKWISE;
                        rotations = 0.125;
                        break;
                    case RED:
                        direction = Direction.CLOCKWISE;
                        rotations = 0.125;
                        break;
                    case YELLOW:
                        direction = Direction.CLOCKWISE;
                        rotations = 0.25;
                        break;
                    default:
                        direction = Direction.CLOCKWISE;
                        rotations = 0;
                }
            break;
            case YELLOW:
                switch (initial) {
                    case BLUE:
                        direction = Direction.CLOCKWISE;
                        rotations = 0.125;
                        break;
                    case GREEN:
                        direction = Direction.CLOCKWISE;
                        rotations = 0.25;
                        break;
                    case RED:
                        direction = Direction.COUNTERCLOCKWISE;
                        rotations = 0.125;
                        break;
                    default:
                        direction = Direction.CLOCKWISE;
                        rotations = 0;
                }
            break;
            default:
                direction = Direction.CLOCKWISE;
                rotations = 0;
        }
    }
}
