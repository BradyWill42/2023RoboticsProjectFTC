package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.MainAutoLinear.sleepMil;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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


/**
 * Example OpMode that demonstrates how to use the ObjectDetection helper class for identifying
 * the Default Signal sleeve or a Custom Signal sleeve.
 *
 * To use this class:
 * 1. Ensure your Webcam is attached and labeled "Webcam 1" in the Robot configuration.
 * 2. If using a custom Tensorflow Lite Model, it's been uploaded to the Robot Controller.
 * 3. You've removed @Disabled so it shows up in the list of available OpModes.
 *
 * To preview the result of object detection, you may perform one of two operations:
 * 1. Plug in a TV/Monitor into the HDMI port of the robot controller. Then, upon pressing Init
 *    (NOT PLAY) of your OpMode, you'll see the resulting output on the monitor.
 * 2. Upon pressing Init (NOT PLAY) of your OpMode, select the "3 dots" menu in the upper right hand
 *    corner of the Driver Station app and select "Camera Stream". Then press the image to refresh
 *    what objects are detected.
 *
 * How to install a custom model:
 * 1. Connect to the Robot Controller's WiFi.
 * 2. Navigate to a browser and enter in: http://192.168.43.1:8080/
 * 3. Click "Manage" at the top.
 * 4. Click "Manage TensorFlow Lite Models" at the bottom.
 * 5. Click "Upload Models" and upload your custom TFLite model.
 * 6. Your custom model is now available to set.
 * 7. Uncomment the line "objectDetection.setCustomModel" by removing the "//" at the beginning.
 *
 * Note: The default 2022-2023 PowerPlay signal sleeve will return the following values:
 *  - "1 Bolt", // Parking spot ONE.
 *  - "2 Bulb", // Parking spot TWO.
 *  - "3 Panel" // Parking spot THREE.
 *
 *  For an example TFLite file, get the files here:
 *  https://drive.google.com/drive/folders/1rd-Y1z12VaA1mnMBjJCSKFDSUqOufUGN?usp=sharing
 *  For the Custom Signal Sleeve Template, see here:
 *  https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/signal-sleeve-template.pdf
 *
 *  To create your own Custom Model:
 *  1. Make your own Signal Sleeve from the PDF link above.
 *  2. Take about 50 photos of each side at varying distances (12 to 30 inches).
 *  3. Upload to Roboflow (see Mr. Kirk for login).
 *  4. Annotate the bounding boxes in Roboflow.
 *  5. Export the Dataset to Google Vertex AI format.
 *  6. Login to Google Vertex AI at https://console.cloud.google.com/vertex-ai.
 *  7. Upload the Dataset.
 *  8. Train the Dataset with a minimum hours of 1.
 *  9. Export to Tensorflow Lite file.
 *  10. Upload to the Robot Controller.
 *  11. Test away!
 * NOTE: A bug was discovered when using Google Colab and exporting to Tensorflow Lite, thus
 * Google's Vertex AI is the only working solution corrently (as of 10/31/2022).
 */
@Autonomous(name="ObjectDetection Right", group="Iterative Opmode")
//@Disabled // NOTE: Remove @Disabled to show up in the list of available opModes.
public class AutoObjectDetection extends LinearOpMode {
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
    public RevColorSensorV3 colorSensor;
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

    public boolean alignClose(){
        boolean aligned = false;
        double maxDistance = distanceSensor.getDistance(INCH);
        while (distanceSensor.getDistance(INCH) > 7 && !aligned) {
            maxDistance = distanceSensor.getDistance(INCH);
            drivetrain.drive(0, -0.3, 0);
            telemetry.addData("DistanceToTarget", distanceSensor.getDistance(INCH));
            if (distanceSensor.getDistance(INCH) <= 7) {
                drivetrain.drive(0, 0, 0);
                aligned = true;
            }
            if (distanceSensor.getDistance(INCH) > 100) {
                drivetrain.drive(0, 0, 0);
                aligned = true;
            }
        }
        return aligned;
    }

    public boolean alignForward(){
        boolean aligned = false;
        double maxDistance = distanceSensor.getDistance(INCH);
        while (distanceSensor.getDistance(INCH) > 3 && !aligned){
            maxDistance = distanceSensor.getDistance(INCH);
            drivetrain.drive(-0.5, 0, 0);
            telemetry.addData("DistanceToTarget", distanceSensor.getDistance(INCH));
            if(distanceSensor.getDistance(INCH) < 3) {
                drivetrain.drive(0, 0, 0);
                aligned = true;
            }
        }
        return aligned;
    }

    public boolean align(){
        boolean aligned = false;
        double maxDistance = distanceSensor.getDistance(INCH);
        while(distanceSensor.getDistance(INCH) > 15 && !aligned){
            maxDistance = distanceSensor.getDistance(INCH);
            drivetrain.drive(0, -0.5, 0);
            telemetry.addData("DistanceToTarget", distanceSensor.getDistance(INCH));
            if(distanceSensor.getDistance(INCH) <= 15) {
                drivetrain.drive(0, 0, 0);
                aligned = true;
            }
        }
        return aligned;
    }


    public void autoCone() {
        if(!ran){
            //FIRST PART - PLACING FIRST CONE
            grabber.closeGrabber();
            sleepMil(2500);
//            grabber.up();
//            sleepMil(4050);
//            grabber.off();
            while(grabber.getEncoder() > -2950){
                grabber.uptakeSetUp();
                if(grabber.getEncoder() >= -2950){
                    break;
                }
            }


//            if(voltage > 14){
//
//                ForwardInches fw = new ForwardInches(-0.7, 37, drivetrain, odometry, simpleOdometry);
//                fw.execute();
//                StrafeInches st = new StrafeInches(-0.7, -6.45, drivetrain, odometry, simpleOdometry);
//                st.execute();
//
//                fw.setInches(2);
//                fw.execute();
//
//                sleep(1000);
//                grabber.uptakeSetDown();
////            grabber.uptakeSetPos(-1500);
//                sleep(3000);
//                grabber.openGrabber();
//
//            } else if(voltage > 13.5){
//                ForwardInches fw = new ForwardInches(-0.7, 37, drivetrain, odometry, simpleOdometry);
//                fw.execute();
//                StrafeInches st = new StrafeInches(-0.7, -6.45, drivetrain, odometry, simpleOdometry);
//                st.execute();
//
//                fw.setInches(2);
//                fw.execute();
//
//                sleep(1000);
//                grabber.uptakeSetDown();
////            grabber.uptakeSetPos(-1500);
//                sleep(3000);
//                grabber.openGrabber();
//            } else {
                ForwardInches fw = new ForwardInches(-0.7, 38.6, drivetrain, odometry, simpleOdometry);
                fw.execute();
                StrafeInches st = new StrafeInches(-0.7, -5.5, drivetrain, odometry, simpleOdometry); //6.5 before
                st.execute();
                align();
                alignClose();


                fw.setInches(1.5);
                fw.execute();

                sleep(1000);
                grabber.uptakeSetDown();
//            grabber.uptakeSetPos(-1000);
                sleep(3000);
                grabber.openGrabber();

//                fw.setInches(-3.5);
//                fw.setPower(0.4);
//                fw.execute();
//
//                TurnAngle ta = new TurnAngle(0.5, -90, drivetrain, odometry, simpleOdometry, gyro);
//                ta.execute();
//
//                fw.setInches(20);
//                fw.setPower(-0.7);
//                fw.execute();
//
//                grabber.closeGrabber();
//                sleep(2000);
//                grabber.uptakeSetUp();
//
//                fw.setInches(-20);
//                fw.setPower(0.7);
//                fw.execute();
//
//                ta.resetAngle();
//                ta.setAngle(179);
//                ta.setPower(0.55);
//                ta.execute();
//
//                grabber.uptakeSetDown();
//                grabber.openGrabber();





            //SECOND PART - PLACING SECOND CONE
//            forwardInches(0.7, -1.95);//-0.75*2.6);
//            strafeInches(0.7, 4.94);//1.9*2.6);
//            drivetrain.drive(0,0,0.7);
//            sleepMil(1125);
//            drivetrain.drive(0,0,0);
//            sleepMil(100);
//            forwardInches(-0.7, 13.52);//5.2*2.6);
//            sleep(1500);
//            grabber.down();
//            sleepMil(1200);
//            grabber.off();
//            grabber.closeGrabber();
//            sleep(3000);
//            grabber.up();
//            sleep(2400);
//            grabber.off();
//            sleep(3000);
//            forwardInches(0.7, 13.65);//-5.25*2.6);
//            sleep(100);
//            drivetrain.drive(0,0,0.7);
//            sleepMil(1150);
//            drivetrain.drive(0,0,0);
//            sleepMil(100);
//            strafeInches(0.7, 7.54);//2.9*2.6);
//            sleepMil(100);
//            forwardInches(-0.7, 1.3);//0.5*2.6);
//            sleepMil(1000);
//            grabber.down();
//            sleepMil(1000);
//            grabber.off();
//            grabber.openGrabber();
//            sleep(100000);
        }
    }

    public void station1() {
        if(!ran){
            autoCone();
            ForwardInches fw = new ForwardInches(0.7, -1, drivetrain, odometry, simpleOdometry);
            fw.execute();
            StrafeInches st = new StrafeInches(-0.7, -5.5, drivetrain, odometry, simpleOdometry);
            st.execute();
            ran = true;
        }

    }
    public void station2() {
        if(!ran) {
            autoCone();
            ForwardInches fw = new ForwardInches(0.7, -1, drivetrain, odometry, simpleOdometry); //0.4 before, test for park between
            fw.execute();
            StrafeInches st = new StrafeInches(0.7, 5.3, drivetrain, odometry, simpleOdometry);
            st.execute();
            ran = true;
        }

    }
    public void station3() {
        if(!ran) {
            autoCone();
            ForwardInches fw = new ForwardInches(0.7, -1, drivetrain, odometry, simpleOdometry); //0.4 before, ^^
            fw.execute();
            StrafeInches st = new StrafeInches(0.7, 16, drivetrain, odometry, simpleOdometry);
            st.execute();
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
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

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

        grabber.isPressed();

        // Wait for the game to start (driver presses PLAY).

        // Example with breaking the loop after 5 attempts.
        while (!isStarted() && !isStopRequested()) {
            labelOfObjectFound = objectDetection.detectObjectAndGetLabel();
            sleep(50); // Wait a little bit before trying again.
            telemetry.addData("OBJ DETECT Object", labelOfObjectFound);
        }

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

//            // Example with breaking the loop after 5 attempts.
//            int counter = 0;
//            while (labelOfObjectFound == null || counter < 5) {
//                labelOfObjectFound = objectDetection.detectObjectAndGetLabel();
//                counter++;
//                sleep(100); // Wait a little bit before trying again.
//            }

            // Determine the parking spot identified. There's lots of ways to do this, but
            // for simplicity, we'll use if/elseif.
             if(labelOfObjectFound == null){
                parkingSpot = 2;
            } else if(labelOfObjectFound.equals("square")) {
                parkingSpot = 1;
            } else if (labelOfObjectFound.equals("circle")) {
                parkingSpot = 2;
            } else if (labelOfObjectFound.equals("triangle")) {
                parkingSpot = 3;
            } else {
                parkingSpot = 2;
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