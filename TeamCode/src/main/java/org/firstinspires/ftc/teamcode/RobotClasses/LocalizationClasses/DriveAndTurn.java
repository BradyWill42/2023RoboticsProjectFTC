package org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainAutoLinear;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.ObjectDetection;

public class DriveAndTurn {
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
    double angle;

    public DriveAndTurn(double power, double inches, double angle, MecanumDrive drivetrain, Odometry odometry, SimpleOdometry simpleOdometry, Gyro gyro)
    {
        this.power = power;
        this.inches = inches;
        this.angle = angle;
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.simpleOdometry = simpleOdometry;
        this.gyro = gyro;

    }

}
