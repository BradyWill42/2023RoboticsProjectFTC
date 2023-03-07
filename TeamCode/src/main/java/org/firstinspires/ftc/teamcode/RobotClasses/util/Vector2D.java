package org.firstinspires.ftc.teamcode.RobotClasses.util;

/**
 * A class for (x, y) coordinates. Any unit can be used to represent x and y but
 * inches are used in the this projects Odometry classes
 */
public class Vector2D {
    private double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public String toString() {
        return "X: " + x + "/ Y: " + y;
    }

    /**
     * Method taken from <a href = "https://github.com/wpilibsuite/allwpilib/blob/45590eea220923903606e47c7428939c3ce979e2/wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/Vector2d.java#L33">
     *     this doc</a> (WPI Lib).
     *     <hr>
     *         Rotate a vector in Cartesian space.
     * @param angle angle in degrees by which to rotate vector counter-clockwise.
     */
    public void rotate(double angle) {
        double cosA = Math.cos(angle * (Math.PI / 180.0));
        double sinA = Math.sin(angle * (Math.PI / 180.0));
        double[] out = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        x = out[0];
        y = out[1];
    }

}
