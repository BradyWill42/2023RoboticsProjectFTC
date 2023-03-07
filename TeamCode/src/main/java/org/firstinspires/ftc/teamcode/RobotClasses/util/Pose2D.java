package org.firstinspires.ftc.teamcode.RobotClasses.util;

/**
 * A class for (x, y) coordinates and rotation of the robot.
 * Any unit can be used to represent x, y, and the angle but inches and degrees are used in
 * the Odometry classes in this project.
 */
public class Pose2D {
    private double x, y, angle;

    public Pose2D(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public Pose2D(Vector2D vector2D, double angle) {
        x = vector2D.getX();
        y = vector2D.getY();
        this.angle = angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getRotation() {
        return angle;
    }

    public String toString() {
        return "X: " + x + "/ Y: " + y + "/ Angle: " + angle;
    }
}
