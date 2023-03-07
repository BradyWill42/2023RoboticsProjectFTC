package org.firstinspires.ftc.teamcode.RobotClasses.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotClasses.util.Vector2D;

/**
 * Drivetrain specs:
 *  The HD Hex Motors with the UltraPlanetary GearBox Kit has a free speed of 6000 RPM.
 *  The gear ratio on our drivetrain is 15:1 so the max theoretical free speed is 400 RPM.
 *  The drivetrain's wheel diameter is 96 mm (3.77953 in)
 *  So the theoretical free speed of our robot is 2.01087 m/s (79.1681 in/s)
 */

/**
 * Class for simple control of a mecanum drivetrain.
 */
public class MecanumDrive  {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private Gyro gyro;

    private double maxSpeedCap;
    private double speedMultiplier;

    public MecanumDrive(DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, Gyro gyro) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;

        this.gyro = gyro;

        maxSpeedCap = 1;
        speedMultiplier = 1;
    }

    /**
     * Control the mecanum drivetrain using forward, strafe, and turn parameters
     * @param forward move forward (positive value) or backward (negative value)
     * @param strafe move right (positive) or left (negative)
     * @param turn turn clock-wise (positive) or counter-clockwise (negative)
     */
    public void drive(double forward, double strafe, double turn, boolean isFieldOriented) {
        double frontLeftPower, backLeftPower, frontRightPower, backRightPower;

        double y = -forward;
        double x = strafe;
        double rx = turn;

        // Field oriented math
        if(isFieldOriented) {

            /* Old field oriented code
            double angle = Math.toRadians(gyro.getAngle());
            double temp = y * Math.cos(angle) + x * Math.sin(angle);
            x = y * Math.sin(angle) + -x * Math.cos(angle);
            y = temp;
             */
            Vector2D input = new Vector2D(x, y);
            input.rotate(-gyro.getAngle());

            // frontLeftPower = input.getX() + input.getY() + rx;
            // frontRightPower = -input.getX() + input.getY() - rx;
            // backLeftPower = -input.getX() + input.getY() + rx;
            // backRightPower = input.getX() + input.getY() - rx;

            frontLeftPower = -input.getY() - input.getX() - rx;
            backLeftPower = input.getY() - input.getX() + rx; //problematic *virtual middle finger*
            frontRightPower = -input.getY() + input.getX() + rx;
            backRightPower = input.getY() + input.getX() - rx;
        }
        else {

            y = -forward; // this is reversed
            x = strafe;
            rx = turn;

            // Set motor power
            frontLeftPower = (-y - x- rx) / 1.5;
            backLeftPower = (y - x + rx) / 1.5;
            frontRightPower = (-y + x + rx) / 1.5; //1.4 for possible belt problem idk what you did brady
            backRightPower = (y + x - rx) / 1.5;



        }

        // if one of the powers is over 1 (or maxSpeedCap), divide them by the max so that all motor powers stay the same ratio
        // (so that they're not over 1 or the maxSpeedCap)
        if (Math.abs(frontLeftPower) > maxSpeedCap || Math.abs(backLeftPower) > maxSpeedCap ||
                Math.abs(frontRightPower) > maxSpeedCap || Math.abs(backRightPower) > maxSpeedCap ) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

//        double frontRightBeltMultiplier = 1.05;

        frontLeft.setPower(frontLeftPower * speedMultiplier);
        backLeft.setPower(backLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier); //* frontRightBeltMultiplier); //Multiplication of 1.25 is due to motor belt falling behind
        backRight.setPower(backRightPower * speedMultiplier);
    }

    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, false);
    }

    /**
     * Cap the power given to the drivetrain
     * @param cap a double from 0 to 1
     */
    public void setSpeedCap(double cap) {
        if(cap > 1) {
            System.out.println("WARNING: Cannot set drivetrain speed cap over 1. Cap has been automatically set to 1");
        }
        maxSpeedCap = Range.clip(Math.abs(cap), 0, 1);
    }

    /**
     * Multiply the speed of all the motors (this is applied after the speed cap is applied)
     * @param multiplier a double from 0 to 1
     */
    public void setSpeedMultiplier(double multiplier) {
        if(multiplier > 1) {
            System.out.println("WARNING: Cannot set drivetrain speed multiplier over 1. Multiplier has been automatically set to 1");
        }
        speedMultiplier = Range.clip(Math.abs(multiplier), 0, 1);
    }

    public void driveStraight(double forward, double strafe) {
        double setpoint = gyro.getAngle();
        double offset = setpoint - (gyro.getAngle() * .08);

        drive(forward, strafe, 0);//, offset);

    }


}
