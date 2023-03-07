package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.MainAutoLinear.sleepMil;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.DistanceSensor;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.DriveAndTurn;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.ForwardInches;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.Odometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.StrafeInches;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.TurnAngle;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.ObjectDetection;

import java.util.ArrayList;

@Autonomous(name="ObjectDetection ParkLeft", group="Iterative Opmode")

public class AutoObjectDetectionPark2 extends LinearOpMode {
    /// Create a variable of the ObjectDetection class.
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
    public DcMotorEx voltage;

    public DcMotor intakeMotor, uptakeMotor1, uptakeMotor2;
    public DcMotorEx shooterMotor;
    public Servo indexer, grabberServo;
    public TouchSensor touchSensor;
    public Rev2mDistanceSensor distanceSensor;
    public Grabber grabber;

    public BNO055IMU imu;

    public MecanumDrive drivetrain;
    //public Intake intake;
    //public Shooter shooter;
    public Odometry odometry;
    public SimpleOdometry simpleOdometry;

    public TrajectoryCommand traj;
    public Gyro gyro;



    /// The name of the label for the object found. Null if not found.
    /// The default signal sleeve will see strings:
    /// "1 squre" - Equals parking spot ONE.
    /// "2 circle" - Equals parking spot TWO.
    /// "3 triangle" - Equals parking spot THREE.
    private String labelOfObjectFound;

    public void forwardInches(double power, double inches) {
        odometry.update();
        simpleOdometry.update();
        double currentY = simpleOdometry.getCurrentVector2D().getY();
        if(inches > 0) {
            odometry.update();
            simpleOdometry.update();
            while (simpleOdometry.getCurrentVector2D().getY() < currentY + inches) {
                odometry.update();
                simpleOdometry.update();
                drivetrain.drive(power, 0, 0, false);
            }
            drivetrain.drive(0, 0, 0);
        }
        else {
            while(simpleOdometry.getCurrentVector2D().getY() > currentY + inches) {
                odometry.update();
                simpleOdometry.update();
                drivetrain.drive(power, 0, 0, false);
            }
            drivetrain.drive(0, 0, 0);
        }
    }

    public void strafeInches(double power, double inches) {
        odometry.update();
        simpleOdometry.update();
        double currentX = simpleOdometry.getCurrentVector2D().getX();
        if(inches > 0) {
            odometry.update();
            simpleOdometry.update();
            while(simpleOdometry.getCurrentVector2D().getX() < currentX + inches) {
                odometry.update();
                simpleOdometry.update();
                drivetrain.drive(0, power, 0, false);
            }
            drivetrain.drive(0, 0, 0);
        }
        else {
            odometry.update();
            simpleOdometry.update();
            while(simpleOdometry.getCurrentVector2D().getX() > currentX + inches) {
                odometry.update();
                simpleOdometry.update();
                drivetrain.drive(0, power, 0, false);
            }
            drivetrain.drive(0, 0, 0);
        }
        odometry.update();
        simpleOdometry.update();
    }

    public void turnAngle(double power, double angle) {
        odometry.update();
        simpleOdometry.update();
        double currentAngle = gyro.getAngle();
        if(angle > 0) {
            while(gyro.getAngle() < currentAngle + angle) {
                drivetrain.drive(0, 0, power);
            }
            drivetrain.drive(0, 0, 0);
        }
        else {
            while(gyro.getAngle() > currentAngle + angle) {
                drivetrain.drive(0, 0, power);
            }
            drivetrain.drive(0, 0, 0);
        }
    }

    public void align(){
        double maxDistance = distanceSensor.getDistance(INCH);
        while(distanceSensor.getDistance(INCH) <= maxDistance){
            maxDistance = distanceSensor.getDistance(INCH);
            //drivetrain.drive(0, -0.7, 0);
            telemetry.addData("Align LEft", distanceSensor.getDistance(INCH));

        }
        while(distanceSensor.getDistance(INCH) >= maxDistance){
            maxDistance = distanceSensor.getDistance(INCH);
            //drivetrain.drive(0, 0.7, 0);
            telemetry.addData("Align Right", distanceSensor.getDistance(INCH));

        }
    }

    public void autoAlign() {

        double currentDistance = distanceSensor.getDistance(INCH);
        double minDistance = 360.0;
        double sendRobotToOdomX;
        double sendRobotToOdomY;

        telemetry.addData("Distance Found", distanceSensor.getDistance(INCH));

        ArrayList<DistanceSensor> array = new ArrayList<DistanceSensor>();
        int i = 0;
        drivetrain.drive(0,-0.7,0);
        while(true) {
            currentDistance = distanceSensor.getDistance(INCH);
            if(currentDistance + 0.35 < minDistance) {
                minDistance = currentDistance;
                array.add(new DistanceSensor(currentDistance, odometry.getCurrentPose().getX(), odometry.getCurrentPose().getY()));
                i++;
            }else if(currentDistance - 0.35 > minDistance) {
                sendRobotToOdomX = array.get(i-1).getOdometryValX();
                sendRobotToOdomY = array.get(i-1).getOdometryValY();
                break;
            }
        }

        while(odometry.getCurrentPose().getX() < sendRobotToOdomX) {
            drivetrain.drive(0,0.7,0);
        }

        drivetrain.drive(0,0,0);

        //strafeInches(0.7, odometry.getCurrentPose().getX() - sendRobotToOdomX);
    }

    public void station1() {
        if(!ran){
//            align();
            grabber.up();
            sleep(300);
            grabber.off();
            sleepMil(3200);
            StrafeInches st = new StrafeInches(-0.7, -10.75, drivetrain, odometry, simpleOdometry);
            st.execute();
            ForwardInches fw = new ForwardInches(-0.7,  27, drivetrain, odometry, simpleOdometry);
            fw.execute();
//            StrafeInches st = new StrafeInches(-0.7, -12, drivetrain, odometry, simpleOdometry);
//            st.execute();
            sleep(3000);
            ran = true;
        }

    }
    public void station2() {
        if(!ran) {
            grabber.up();
            sleep(300);
            grabber.off();
            sleepMil(3200);
            ForwardInches fw = new ForwardInches(-0.7, 27, drivetrain, odometry, simpleOdometry);
            fw.execute();
            sleep(3000);
            ran = true;
        }

    }
    public void station3() {
        if(!ran) {
            grabber.up();
            sleep(300);
            grabber.off();
            sleepMil(3200);
            ForwardInches fw = new ForwardInches(-0.7, 2.5, drivetrain, odometry, simpleOdometry);
            fw.execute();
            StrafeInches st = new StrafeInches(0.7, 10, drivetrain, odometry, simpleOdometry);
            st.execute();
            ForwardInches fwrt = new ForwardInches(-0.7, 27, drivetrain, odometry, simpleOdometry);
            fwrt.execute();
//            StrafeInches st = new StrafeInches(0.7, 9, drivetrain, odometry, simpleOdometry);
//            st.execute();
            sleep(3000);
            ran = true;
        }
    }

    @Override
    public void runOpMode() {
        // Setup object detection.

        objectDetection = new ObjectDetection(hardwareMap, telemetry);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRight = hardwareMap.get(DcMotor.class, "backRightDrive");

        leftOdometer = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        rightOdometer = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backOdometer = hardwareMap.get(DcMotor.class, "backLeftDrive");

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
        uptakeMotor1 = hardwareMap.get(DcMotor.class, "uptakeMotor1");
        uptakeMotor2 = hardwareMap.get(DcMotor.class, "uptakeMotor2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        voltage = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");

        //indexer = hardwareMap.get(Servo.class, "indexerServo");
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");

        gyro = new Gyro(imu);
        drivetrain = new MecanumDrive(frontRight, frontLeft, backRight, backLeft, gyro);
//        intake = new Intake(intakeMotor);
//        shooter = new Shooter(shooterMotor, indexer);
        grabber = new Grabber(uptakeMotor1, uptakeMotor2, grabberServo, touchSensor);
        odometry = new Odometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);
        simpleOdometry = new SimpleOdometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);

        traj = new TrajectoryCommand(odometry, drivetrain);


        // If you have a custom model, set the file and labels.
        // Note: There is not requirement as to the order of labels in the list.
        // If you'd like to test out the below TFLite file, see the files here:
        // https://drive.google.com/drive/folders/1rd-Y1z12VaA1mnMBjJCSKFDSUqOufUGN?usp=sharing
        String[] myCustomLabels = { "circle", "triangle", "square" };
        objectDetection.setCustomModel("SignalSleeveDetectionIMPROVED.tflite", myCustomLabels);

        // Set the confidence level for the objects we want to find. The higher
        // the confidence level, the more accuracy is required. It is not recommended
        // to exceed 0.8 (80%) for the robot. Lowering accuracy might cause the detection
        // to provide an incorrect identification.
        objectDetection.minimumConfidenceThreshold = 0.75;
        // Activate object detection. Once activated, it starts recognizing objects.
        objectDetection.activateIfNeeded();

        // Log out that the robot has been initialized.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Gyro Angle", gyro.getAngle());
        telemetry.addData("Angle", gyro.getAngle());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).
        waitForStart();

        // Run until the end of the match (driver presses STOP).
        // Note: This object detection will run repeatedly, since this is a standard
        // OpMode and not autonomous. Thus, you can move the signal around and validate
        // the output as much as you'd like.
        while (opModeIsActive()) {

            // Set a variable for the parking spot identified.
            int parkingSpot = 0;

            odometry.update();
            simpleOdometry.update();
            telemetry.addData("Odometry", "X Value (in): " + simpleOdometry.getCurrentVector2D().getX());
            telemetry.addData("Odometry", "Y Value (in): " + simpleOdometry.getCurrentVector2D().getY());

            telemetry.addData("odometry norm", "x val: " + odometry.getCurrentPose().getX());
            telemetry.addData("odometry norm", "y val: " + odometry.getCurrentPose().getY());
            // Create a while loop to keep looking for an object until one is found. This loop
            // will run over and over, and potentially forever. It would be recommended to create
            // a fail-safe whereby it gives up and uses a default object after a period of time.
            // while (labelOfObjectFound == null) {
            //     labelOfObjectFound = objectDetection.detectObjectAndGetLabel();
            //     sleep(100); // Wait a little bit before trying again.
            // }

            // Example with breaking the loop after 5 attempts.
            int counter = 0;
            while (labelOfObjectFound == null || counter < 5) {
                labelOfObjectFound = objectDetection.detectObjectAndGetLabel();
                counter++;
                sleep(100); // Wait a little bit before trying again.
            }

            // Determine the parking spot identified. There's lots of ways to do this, but
            // for simplicity, we'll use if/elseif.
            if (labelOfObjectFound.equals("square")) {
                parkingSpot = 1;
            } else if (labelOfObjectFound.equals("circle")) {
                parkingSpot = 2;
            } else if (labelOfObjectFound.equals("triangle")) {
                parkingSpot = 3;
            } else {
                parkingSpot = 2; // Fail-safe if we didn't find what we expected.
            }

            // Log which parking spot and label was found:
            telemetry.addData("Seen Label", labelOfObjectFound);
            telemetry.addData("Parking Spot", parkingSpot);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Odometry", "X Value (in): " + simpleOdometry.getCurrentVector2D().getX());
            telemetry.addData("Odometry", "Y Value (in): " + simpleOdometry.getCurrentVector2D().getY());

            telemetry.addData("odometry norm", "x val: " + odometry.getCurrentPose().getX());
            telemetry.addData("odometry norm", "x val: " + odometry.getCurrentPose().getY());
            // Wait a second before proceeding so you have time to read the log output.
            // Ensure you aren't using sleep(); in your final opModes for competition or practice.
            sleep(1000);

            //
            // Do something with the Parking Spot identified.
            //

            if(parkingSpot == 1) {
                odometry.update();
                simpleOdometry.update();
                station1();
            }else if(parkingSpot == 2) {
                odometry.update();
                simpleOdometry.update();
                station2();
            }else if(parkingSpot == 3) {
                odometry.update();
                simpleOdometry.update();
                station3();
            }

            // Reset the variable so this loop will repeat. Ensure you remove this for
            // competition or practice as it might cause issues.
            labelOfObjectFound = null;

            // Output log that the robot is running mode.
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
