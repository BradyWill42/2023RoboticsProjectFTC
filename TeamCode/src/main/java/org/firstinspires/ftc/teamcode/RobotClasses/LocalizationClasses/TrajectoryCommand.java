package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.util.PIDController;
import org.firstinspires.ftc.teamcode.RobotClasses.util.Pose2D;

import java.util.ArrayList;

public class TrajectoryCommand {
    private ArrayList<Pose2D> trajectory;
    private Odometry odometry;
    private MecanumDrive drivetrain;

    private double drivePower, rotatePower;

    public TrajectoryCommand(Odometry odometry, MecanumDrive drivetrain) {
        this.odometry = odometry;
        this.drivetrain = drivetrain;
        drivePower = .4;
        rotatePower = .4;
    }

    public void buildTrajectory(ArrayList<Pose2D> trajectory) {
        this.trajectory = trajectory;
    }

    public boolean atX(Pose2D targetWaypoint) {
        double currentX = odometry.getCurrentPose().getX();
        double targetX = targetWaypoint.getX();
        return currentX > targetX - 1.5 && currentX < targetX + 1.5;
    }

    public boolean atY(Pose2D targetWaypoint) {
        double currentY = odometry.getCurrentPose().getY();
        double targetY = targetWaypoint.getY();
        return currentY > targetY - 1.5 && currentY < targetY + 1.5;
    }

    public boolean atRotation(Pose2D targetWaypoint) {
        double currentRotation = odometry.getCurrentPose().getRotation();
        double targetRotation = targetWaypoint.getRotation();
        return currentRotation > targetRotation - 4 && currentRotation < targetRotation + 4;
    }

    public boolean atWaypoint(Pose2D targetWaypoint) {
        return atX(targetWaypoint) && atY(targetWaypoint) && atRotation(targetWaypoint);
    }

    public void setDrivePower(double power) {
        drivePower = power;
    }

    public void setRotatePower(double power) {
        rotatePower = power;
    }

    public void goToPoint(Pose2D targetWaypoint) {
        PIDController forwardPID = new PIDController(.08, .001, .01);
        PIDController strafePID = new PIDController(.08, .001, .01);
        PIDController rotatePID = new PIDController(.01, 1, .001);

        forwardPID.setOutputRange(-drivePower, drivePower);
        strafePID.setOutputRange(-drivePower, drivePower);
        rotatePID.setOutputRange(-rotatePower, rotatePower);

        forwardPID.setDt(1);
        strafePID.setDt(1);

        double forward, strafe, rotate;

        if(!atX(targetWaypoint)) {
            forward = -forwardPID.calculate(odometry.getCurrentPose().getX(), targetWaypoint.getX());
        }
        else {
            forward = 0;
        }

        if(!atY(targetWaypoint)) {
            strafe = -strafePID.calculate(odometry.getCurrentPose().getY(), targetWaypoint.getY());
        }
        else {
            strafe = 0;
        }

        if(!atRotation(targetWaypoint)) {
            rotate = -rotatePID.calculate(odometry.getCurrentPose().getRotation(), targetWaypoint.getRotation()) * .6;
        }
        else {
            rotate = 0;
        }

        drivetrain.drive(forward, strafe, rotate);
        odometry.update();


    }

}
