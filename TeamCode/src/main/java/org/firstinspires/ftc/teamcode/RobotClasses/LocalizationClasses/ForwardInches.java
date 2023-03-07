package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainAutoLinear;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.Odometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.ObjectDetection;

public class ForwardInches {
    private ObjectDetection objectDetection;
    private MainAutoLinear station;
    private boolean ran = false;
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor leftOdometer, rightOdometer, backOdometer;

    public DcMotor intakeMotor, uptakeMotor;
    public DcMotorEx shooterMotor;
    public Servo indexer, grabberServo;
    public TouchSensor touchSensor;
    public Grabber grabber;

    public BNO055IMU imu;

    public MecanumDrive drivetrain;
    //public Intake intake;
    //public Shooter shooter;
    public Odometry odometry;
    public SimpleOdometry simpleOdometry;

    public TrajectoryCommand traj;
    public Gyro gyro;
    double power;
    double inches;
    double currentPower = 0;

    public ForwardInches(double power, double inches, MecanumDrive drivetrain, Odometry odometry, SimpleOdometry simpleOdometry) {
        this.power = power;
        this.inches = inches;
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.simpleOdometry = simpleOdometry;
    }

    public void setPower(double power){
        this.power = power;
    }

    public void setInches(double inches){
        this.inches = inches;
    }

    public void execute() {
        odometry.update();
        simpleOdometry.update();
        double initialPos = simpleOdometry.getCurrentVector2D().getY();
        if(inches > 0) {
            odometry.update();
            simpleOdometry.update();
            while (simpleOdometry.getCurrentVector2D().getY() < initialPos + inches) {
                odometry.update();
                simpleOdometry.update();
                currentPower = currentPower * 0.85 + power * 0.15;
                drivetrain.drive(currentPower, 0, 0, false);
//                if(simpleOdometry.getCurrentVector2D().getY() < (initialPos + inches)/2){
//                    power = 0;
//                }
            }
            drivetrain.drive(0, 0, 0);
        }
        else {
            while (simpleOdometry.getCurrentVector2D().getY() > initialPos + inches) {
                odometry.update();
                simpleOdometry.update();
                currentPower = currentPower * 0.85 + power * 0.15;
                drivetrain.drive(currentPower, 0, 0, false);
//                if(simpleOdometry.getCurrentVector2D().getY() < (initialPos + inches)/2){
//                    power = 0;
//                }
            }
            drivetrain.drive(0, 0, 0);
        }
    }



}
