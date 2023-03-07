package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;

public class DistanceSensor {

    private double distanceAway;
    private double odometryValX;
    private double odometryValY;

    public DistanceSensor(double distanceAway, double odometryValX, double odometryValY) {
        this.distanceAway = distanceAway;
        this.odometryValX = odometryValX;
        this.odometryValY = odometryValY;
    }

    public double getDistanceAway() {
        return distanceAway;
    }

    public double getOdometryValX() {
        return odometryValX;
    }

    public double getOdometryValY() {
        return odometryValY;
    }

}
