package org.firstinspires.ftc.teamcode.helper;

/**
 * Some simple classes for holding geometry-related data.
 * There are already several other classes for this purpose, but these are very simple.
 */
public class Geo {
    public static class Vector2D {
        public final double x;
        public final double y;

        public Vector2D(double x, double y) {
            this.x = x;
            this.y = y;
        }

        /**
         * @return A new Vector2D representing this vector rotated in Cartesian space
         */
        public Vector2D rotatedBy(double radians) {
            return rotateBy(x, y, radians);
        }

        /**
         * @return A new Vector2D representing the coordinates rotated in Cartesian space
         */
        public static Vector2D rotateBy(double x, double y, double radians) {
            double cosA = Math.cos(radians);
            double sinA = Math.sin(radians);
            double rX = x * cosA - y * sinA;
            double rY = x * sinA + y * cosA;
            return new Vector2D(rX, rY);
        }
    }

    public static class Orientation2D {
        public final double angle;
        public final double matrixX;
        public final double matrixY;

        public Orientation2D(double angle) {
            this.angle = angle;
            this.matrixX = -Math.sin(angle);
            this.matrixY = Math.cos(angle);
        }
    }

    public static class Pose2D {
        public final double x;
        public final double y;
        public final double heading;

        public Pose2D(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
}
