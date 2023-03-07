package org.firstinspires.ftc.teamcode;

        import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

        import android.graphics.Paint;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
        import com.qualcomm.hardware.rev.RevColorSensorV3;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import com.qualcomm.robotcore.hardware.VoltageSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import static org.firstinspires.ftc.teamcode.MainAutoLinear.sleepMil;

        import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.DistanceSensor;
        import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.Odometry;
        import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;
        import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.TrajectoryCommand;
        import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Grabber;
        import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
        import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
        import org.firstinspires.ftc.teamcode.RobotClasses.util.Pose2D;

        import java.util.ArrayList;

@TeleOp(name="Funny Teleop", group="Iterative Opmode")
public class FunnyTeleop extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor leftOdometer, rightOdometer, backOdometer;

    private DcMotor intakeMotor, uptakeMotor1, uptakeMotor2;
    private DcMotorEx shooterMotor;
    private Servo indexer, grabberServo;
    private RevColorSensorV3 colorSensor;
    private Rev2mDistanceSensor distanceSensor;
    private TouchSensor touchSensor;

    private BNO055IMU imu;

    private MecanumDrive drivetrain;
    //private Intake intake;
    //private Shooter shooter;
    private Grabber grabber;

    private Gyro gyro;

    private Odometry odometry;
    private SimpleOdometry simpleOdometry;

    private TrajectoryCommand traj;

    boolean x_boolean, y_boolean;

    private double forward;
    private double strafe;
    private double turn;
    private double desiredStrafe;
    private double desiredForward;
    private double desiredTurn;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        //colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRight = hardwareMap.get(DcMotor.class, "backRightDrive");

        leftOdometer = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        rightOdometer = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backOdometer = hardwareMap.get(DcMotor.class, "backLeftDrive");




//        deadwheelMiddle = new Encoder(hardwareMap.get(DcMotorEx.class, "deadwheelMiddle"));
//
//        Encoder
        // Reset encoders
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set direction of the motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        //shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        //uptakeMotor1 = hardwareMap.get(DcMotor.class, "uptakeMotor1");
        //uptakeMotor2 = hardwareMap.get(DcMotor.class, "uptakeMotor2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //indexer = hardwareMap.get(Servo.class, "indexerServo");
        //indexer.scaleRange(0, 180);
        //grabberServo = hardwareMap.get(Servo.class, "grabberServo");

        //touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        //if you're reading this you're gay
        gyro = new Gyro(imu);
        drivetrain = new MecanumDrive(frontRight, frontLeft, backRight, backLeft, gyro);
        //intake = new Intake(intakeMotor);
        //shooter = new Shooter(shooterMotor, indexer);
        //grabber = new Grabber(uptakeMotor1, uptakeMotor2, grabberServo, touchSensor);

        odometry = new Odometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);
        simpleOdometry = new SimpleOdometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);

        traj = new TrajectoryCommand(odometry, drivetrain);


        // Toggle Booleans
        x_boolean = false;
        y_boolean = false;

        forward = 0;
        strafe = 0;
        turn = 0;



        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    public boolean align(){
        /*boolean aligned = false;
        double maxDistance = distanceSensor.getDistance(INCH);
        if (distanceSensor.getDistance(INCH) > 10){
            maxDistance = distanceSensor.getDistance(INCH);
            drivetrain.drive(0, -0.7, 0);
            telemetry.addData("DistanceToTarget", distanceSensor.getDistance(INCH));
            aligned = false;
        } else if(distanceSensor.getDistance(INCH) <= 10) {
            aligned = true;
        }
        return aligned;*/
        return true;
    }

    // poo poo pee pee
    @Override
    public void loop() {

        odometry.update();
        simpleOdometry.update();



        // Driving
//        double forward = gamepad1.left_stick_y;
//        double strafe = 0;
//        double turn = gamepad1.right_stick_x;
//        double desiredPow = gamepad1.left_stick_x;

//        if(gamepad1.left_stick_x < 0){
//            if (strafe > desiredPow){
//                //strafe = strafe - 0.01;
//                strafe = strafe - 0.01;
//                drivetrain.drive(forward, strafe, turn);
//            }
//        } else if(gamepad1.left_stick_x > 0){
//            if (strafe < desiredPow){
//                //strafe = strafe + 0.01;
//                strafe = strafe + 0.01;
//                drivetrain.drive(forward, strafe, turn);
//            }
//        } else {
//            strafe = 0;
//            drivetrain.drive(forward, strafe, turn);
//        }
        desiredStrafe = gamepad1.left_stick_x;
        desiredForward = gamepad1.left_stick_y;
        desiredTurn = gamepad1.right_stick_x;
        /*if(desiredForward == 0){
            forward = 0;
        } else {
            forward = (0.70 * forward) + (0.30 * desiredForward);
        }

        if(desiredStrafe == 0){
            strafe = 0;
        } else {
            strafe = (0.7 * strafe) + (0.3 * desiredStrafe);
        }*/
        forward = desiredForward;
        strafe = desiredStrafe;

        turn = desiredTurn;

        drivetrain.drive(forward, strafe, turn);



        telemetry.addData("stick x pos", gamepad1.left_stick_x);

        //drivetrain.drive(forward, strafe, turn);


        // Toggle booleans
        if(gamepad1.x || gamepad2.x) {
            x_boolean = !x_boolean;
        }
        if(gamepad1.y || gamepad2.y) {
            y_boolean = !y_boolean;
        }

        // Intake On/Off
//        if(gamepad1.x || gamepad2.x) {
//            intake.on();
//        }
//        else if(gamepad1.b || gamepad2.b) {
//            intake.reverse();
//        }
//        else {
//            intake.off();
//        }

        // Shooter On/Off
//        if(gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
//            shooter.on();
//        }
//        else {
//            shooter.off();
//        }

        // Indexer
//        if(gamepad1.a || gamepad2.a) {
//            shooter.setIndexer(0);
//        }
//        else {
//            shooter.setIndexer(.65);
//        }
/*
        if(gamepad1.dpad_down || gamepad2.dpad_down) {
            grabber.uptakeSetDown();
            //autoAlign();
        }
        else if(gamepad1.dpad_up || gamepad2.dpad_up) {
            grabber.uptakeSetUp();
        } else {
            grabber.off();
        }

        if(gamepad1.dpad_left || gamepad2.left_stick_button) {
            grabber.openGrabber();
        }
        else if(gamepad1.dpad_right || gamepad2.right_stick_button) {
            grabber.closeGrabber();
        }

        if(gamepad1.back || gamepad2.back) {
            //grabber.armSetDown();
            grabber.down();
        }
        else if(gamepad1.start || gamepad2.start) {
            //grabber.armSetUp();
            grabber.up();
        }

        if(gamepad1.right_bumper){
            align();
        }

        if(gamepad2.right_bumper) {
            grabber.upSlow();
        }
        else if(gamepad2.left_bumper) {
            grabber.downSlow();
        }

//        if(gamepad1.back) {
//            odometry.resetPose();
//        }
*/
        //telemetry.addData("Red Sensed", colorSensor.red());
        //telemetry.addData("Blue Sensed", colorSensor.blue());
        //telemetry.addData("Color distance", colorSensor.getDistance(INCH));
//        telemetry.addData("Hue", colorSensor.);
        //telemetry.addData("distance sensor", distanceSensor.getDistance(INCH));
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Simple Odometry", "X Value (in): " + simpleOdometry.getCurrentVector2D().getX());
        telemetry.addData("Simple Odometry", "Y Value (in): " + simpleOdometry.getCurrentVector2D().getY());

        telemetry.addData("Odometry", "X: " + odometry.getCurrentPose().getX());
        telemetry.addData("Odometry", "Y: " + odometry.getCurrentPose().getY());
        telemetry.addData("Odometry", "Angle: " + odometry.getCurrentPose().getRotation());

        telemetry.addData("LeftEncoder", "Val: " + leftOdometer.getCurrentPosition());
        telemetry.addData("RightEncoder", "Val: " + rightOdometer.getCurrentPosition());
        telemetry.addData("BackEncoder", "Val: " + backOdometer.getCurrentPosition());

        telemetry.addData("Angle", gyro.getAngle());

        telemetry.addData("Controller 1 x pos: ", gamepad1.left_stick_x);
        telemetry.addData("Controller 1 y pos: ", gamepad1.left_stick_y);

        //telemetry.addData("Uptake encoder", grabber.getEncoder());

        //telemetry.addData("Touch sensor pressed:", grabber.isPressed());

        //telemetry.addData("uptakeMotor1", uptakeMotor1.getCurrentPosition());
        //telemetry.addData("uptakeMotor2", uptakeMotor2.getCurrentPosition());
        telemetry.addData("controller right button down", gamepad2.right_stick_button);

        telemetry.addData("gyroX", gyro.getPos().x);
        telemetry.addData("gyroY", gyro.getPos().y);
        telemetry.addData("gyroZ", gyro.getPos().z);
        //telemetry.addData("Shooter Servo", indexer.getPosition());
    }

    @Override
    public void stop() {
    }

}