package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.util.Vector2D;

import java.util.ArrayList;

/**
 * Extremely simple localization class for 3 dead-wheel odometry and no turning functionality.
 * NOTE: trajectory/waypoint following will be accurate up to +- 1/2 of an inch.
 */
public class SimpleOdometry {
    MecanumDrive drivetrain;
    DcMotor leftOdometer, rightOdometer, backOdometer;
    Vector2D currentVector2D;
    Gyro gyro;

    double xPos, yPos;

    final double WHEEL_DIAMETER = 3.77953;  //1.49606; // Diameter of the wheel attached to the encoder in inches (equals 38 mm)
    final double CPR = 560; //1440; //counts per (one) revolution of the odometer's encoder shaft

    public SimpleOdometry(MecanumDrive drivetrain, Gyro gyro, DcMotor leftOdometer, DcMotor rightOdometer, DcMotor backOdometer) {
        this.drivetrain = drivetrain;
        this.leftOdometer = leftOdometer;
        this.rightOdometer = rightOdometer;
        this.backOdometer = backOdometer;


        //leftOdometer.setDirection(DcMotor.Direction.REVERSE);

        currentVector2D = new Vector2D(0, 0);
    }

    public double getEncoderDistance(DcMotor odometer) {
            return (odometer.getCurrentPosition() / CPR) * Math.PI * WHEEL_DIAMETER;
    }

    public double calculateForwardDistance() {
        return (getEncoderDistance(leftOdometer) + (getEncoderDistance(rightOdometer))) / 2;
    }

    public double calculateStrafeDistance() {
        return (getEncoderDistance(backOdometer));
    }

    public Vector2D getCurrentVector2D() {
        return currentVector2D;
    }

    public void resetPose() {
        xPos = 0;
        yPos = 0;
    }

    public boolean isAtWaypoint(Vector2D currentWaypoint, Vector2D targetWaypoint) {
        boolean xTrue = currentWaypoint.getX() > targetWaypoint.getX() - .5 && currentWaypoint.getX() < targetWaypoint.getX() + .5;
        boolean yTrue = currentWaypoint.getY() > targetWaypoint.getY() - .5 && currentWaypoint.getY() < targetWaypoint.getY() + .5;
        return xTrue && yTrue;
    }

    /**
     * Updates the odometry. Call this method at the beginning or end of every loop
     */
    public void update() {
        xPos = calculateForwardDistance();
        yPos = calculateStrafeDistance();
        currentVector2D = new Vector2D(xPos, yPos);
    }

    /**
     * Follow a given trajectory.
     * NOTE: the first waypoint is always the current pose. The first waypoint in the given ArrayList trajectory is
     * treated as the second waypoint in the trajectory (the first waypoint the robot will drive towards)
     * @param trajectory an ArrayList of waypoints for the robot to follow
     * @param xPower the "forward" power towards the x direction
     * @param yPower the "strafe" power towards the y direction
     */
    public void followTrajectory(ArrayList<Vector2D> trajectory, double xPower, double yPower) {
        xPower = Math.abs(xPower);
        yPower = Math.abs(yPower);

        for(Vector2D waypoint : trajectory) {

            update();

            xPower = waypoint.getX() > getCurrentVector2D().getX() ? xPower : -xPower;
            yPower = waypoint.getY() > getCurrentVector2D().getY() ? yPower : -yPower;
            // TODO: add gyro correction
            while(!isAtWaypoint(waypoint, getCurrentVector2D())) {

                update();

                if(getCurrentVector2D().getX() > waypoint.getX() - .5 && getCurrentVector2D().getX() < waypoint.getX() + .5) {
                    xPower = 0;
                }
                if(getCurrentVector2D().getY() > waypoint.getY() - .5 && getCurrentVector2D().getY() < waypoint.getY() + .5) {
                    yPower = 0;
                }

                drivetrain.drive(xPower, yPower, 0);
            }

            drivetrain.drive(0, 0, 0);

        }
    }
}

