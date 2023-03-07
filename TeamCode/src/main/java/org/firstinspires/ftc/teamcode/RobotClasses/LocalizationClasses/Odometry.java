package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.util.Pose2D;
import org.firstinspires.ftc.teamcode.RobotClasses.util.Vector2D;
import org.firstinspires.ftc.teamcode.RobotClasses.util.VirusVector2D;

/**
 * Localization class for 3 wheel Odometry. This class is based on Centennial High School FTC (Team VIRUS #9866).
 * <a href = https://github.com/Centennial-FTC-Robotics/Virus2019-2020/blob/master/TeamCode/src/main/java/org/virus/agobot/Odometry.java>
 *     This was the Java document </a>I adapted code from.
 * A derivation of the math from the same team used can
 * be found <a href = https://drive.google.com/file/d/11c3Z9EkDj2_GuOQSFzpVzba_HPKqD6ej/view>here.</a>
 */
public class Odometry extends SimpleOdometry {
    MecanumDrive drivetrain;
    Gyro gyro;

    Pose2D currentPose;

    DcMotor deadWheelLeft, deadWheelRight, deadWheelMiddle;

    double deltaLeftEncoder, deltaRightEncoder, deltaBackEncoder;
    double leftEncoderPrev, rightEncoderPrev, backEncoderPrev;
    double deltaHeading;
    double deltaX, deltaY;

    double heading, startHeading, headingCorrection, prevHeading;
    double xPos, yPos, anglePos;

    final double trackwidth = 16.0; //in inches
    final double length = 17.0; // in inches

    final double ENCODER_COUNTS_PER_INCH = super.CPR / (WHEEL_DIAMETER * Math.PI);
    final double RADIUS = trackwidth / 2.0; // in inches, the distance from the middle of the robot to the left/right odometer
    final double BACK_RADIUS = 0.1; //length / 2.0; //in inches, the distance from the middle of the robot (length-wise) to the back odometer

    Vector2D robotCentricDelta;

    public Odometry(MecanumDrive drivetrain, Gyro gyro, DcMotor deadWheelLeft, DcMotor deadWheelRight, DcMotor deadWheelMiddle) {
        super(drivetrain, gyro, deadWheelLeft, deadWheelRight, deadWheelMiddle);
        this.deadWheelLeft = deadWheelLeft;
        this.deadWheelRight = deadWheelRight;
        this.deadWheelMiddle = deadWheelMiddle;

        this.gyro = gyro;
        this.drivetrain = drivetrain;

        currentPose = new Pose2D(0, 0, gyro.getAngle());

        heading = 0;
        startHeading = 0;
        headingCorrection = 0;
        prevHeading = 0;

        xPos = 0;
        yPos = 0;
        anglePos = 0;
    }

    /**
     * Resets all encoders (duh)
     */
    public void resetAllEncoders() {
        deadWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelMiddle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



    /**
     * Normalize a radian measure to [0, 2pi]
     * @param angleRadians angle measure in radians
     * @return an equivalent angle measure from 0 to 2pi (in radians obv).
     */
    public double normalizeRadians(double angleRadians) {
        while(angleRadians > 2 * Math.PI) {
            angleRadians -= 2 * Math.PI;
        }
        while(angleRadians < 0.0) {
            angleRadians += 2 * Math.PI;
        }
        return angleRadians;
    }

    /**
     *
     * @return current position of left encoder
     */
    public double getLeftPosition() {
        return deadWheelLeft.getCurrentPosition();
    }

    /**
     *
     * @return current position of right encoder (corrected for direction)
     */
    public double getRightPosition() {
        return deadWheelRight.getCurrentPosition();
    }

    /**
     *
     * @return current position of back encoder (correction for direction)
     */
    public double getBackPosition() {
        return (deadWheelMiddle.getCurrentPosition() * -1);
    }

    /**
     * Works much like the getEncoderDistance() method in the SimpleOdometry class
     * but accepts a double rather than an encoder
     * @param count encoder value
     * @return distance the encoder wheel has traveled in inches
     */
    public double countToInch(double count) {
        return (count / super.CPR) * Math.PI * super.WHEEL_DIAMETER;
    }

    public Pose2D getCurrentPose() {
        return currentPose;
    }

    /**
     * Sets the current pose to one specified by the user. NOTE: There is a secondary setPose2D method
     * that accepts a pose rather than it's parameters.
     * @param x
     * @param y
     * @param rotation in degrees
     */
    public void setPose2D(double x, double y, double rotation) {
        currentPose = new Pose2D(x, y, rotation);
    }

    public void setPose2D(Pose2D pose) {
        currentPose = pose;
    }

    public void resetPose() {
        xPos = 0;
        yPos = 0;
    }

    public Vector2D getRobotCentricDelta() {
        return robotCentricDelta;
    }

    public double getDeltaBackEncoder() {
        return deltaBackEncoder;
    }

    public Vector2D getRotatedVector() {
        Vector2D tempVector = new Vector2D(xPos, yPos);
        tempVector.rotate(Math.toDegrees(anglePos));
        return tempVector;
    }

    /**
     * Updates the current pose of the robot
     */
    public void update() {
        deltaLeftEncoder = getLeftPosition() - leftEncoderPrev;
        deltaRightEncoder = getRightPosition() - rightEncoderPrev;
        deltaBackEncoder = getBackPosition() - backEncoderPrev;

        leftEncoderPrev = getLeftPosition();
        rightEncoderPrev = getRightPosition();
        backEncoderPrev = getBackPosition();

        deltaHeading = (deltaRightEncoder - deltaLeftEncoder) / (2.0 * RADIUS * ENCODER_COUNTS_PER_INCH); // in radians

        /*
        heading = Math.toRadians(gyro.getAngle());
        deltaHeading = heading - prevHeading;
        prevHeading = Math.toRadians(gyro.getAngle());

         */

        heading = normalizeRadians((getRightPosition() - getLeftPosition())) / (2.0 * RADIUS * ENCODER_COUNTS_PER_INCH) + startHeading + headingCorrection;

        if(deltaHeading == 0) {
            deltaX = deltaBackEncoder;
            deltaY = (deltaBackEncoder + deltaRightEncoder) / 2.0;
        }
        else {
            double turnRadius = RADIUS * ENCODER_COUNTS_PER_INCH * (deltaLeftEncoder + deltaRightEncoder) / (deltaRightEncoder - deltaLeftEncoder);
            double strafeRadius = deltaBackEncoder / deltaHeading - BACK_RADIUS * ENCODER_COUNTS_PER_INCH;

            deltaX = turnRadius * (Math.cos(deltaHeading) - 1) + strafeRadius * Math.sin(deltaHeading);
            deltaY = turnRadius * Math.sin(deltaHeading) + strafeRadius * (1 - Math.cos(deltaHeading));
        }

        robotCentricDelta = new Vector2D(countToInch(deltaX), countToInch(deltaY));

        anglePos += deltaHeading;

        VirusVector2D tempVector = new VirusVector2D(countToInch(deltaY), countToInch(-deltaX));
        tempVector.rotate(Math.toRadians(gyro.getAngle()));

        xPos += tempVector.getComponent(0);
        yPos += tempVector.getComponent(1);

        currentPose = new Pose2D(xPos, yPos, gyro.getAngle());

    }
}
