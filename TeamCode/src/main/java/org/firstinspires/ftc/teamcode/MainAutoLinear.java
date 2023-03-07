/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.Odometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.SimpleOdometry;
import org.firstinspires.ftc.teamcode.RobotClasses.LocalizationClasses.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.Gyro;
import org.firstinspires.ftc.teamcode.RobotClasses.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotClasses.util.Pose2D;

import java.util.List;

@Autonomous(name="Main: Auto Linear", group="Iterative Opmode")
public class MainAutoLinear extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor leftOdometer, rightOdometer, backOdometer;

    public DcMotor intakeMotor, uptakeMotor1, uptakeMotor2;
    public DcMotorEx shooterMotor;
    public Servo indexer, grabberServo;
    public TouchSensor touchSensor;
    public Grabber grabber;

    public BNO055IMU imu;

    public MecanumDrive drivetrain;
    //public Intake intake;
    //public Shooter shooter;

    public Gyro gyro;

    public Odometry odometry;
    public SimpleOdometry simpleOdometry;

    public TrajectoryCommand traj;

    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";

    public static final String VUFORIA_KEY =
            "AeO4IGj/////AAABmU63rhEa2UHcp6WYD9aPftR0Ng+l2Y1rTyE504+3KOuaDgvQMCW9M+GdjYidkvXsch5FEbYgsCtnACgS/CFcN6ZcJuyGALGfShSJ7+lZC5JOO4muO9G8GtoF+29tsSFLzUloVHHnC7dTpjxkOdJMfBiJWd5BlwVk08ESHIFRg4XoyCTrgkUAzljSW6u3b6uhW+IrtYvcocQMJude0+a8kckI5iN25AabaOcj108frbkVki0uTZzehjG4u2Ve2eUvNY7q7hqd3QtA0gB58K3wYnxpbWkbA/QActi/+ogNBdJb8bNUv1VGi32QDJrNabymUebURoM5fmq91Is9nUltgY5RifOhe8U3b/laxAX+5ZQB";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public Recognition trueRec;


    @Override
    public void runOpMode() {
        //** INIT **//
        telemetry.addData("Status", "Initialized");

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

        //indexer = hardwareMap.get(Servo.class, "indexerServo");
        grabberServo = hardwareMap.get(Servo.class, "grabberServo");

        gyro = new Gyro(imu);
        drivetrain = new MecanumDrive(frontRight, frontLeft, backRight, backLeft, gyro);
//        intake = new Intake(intakeMotor);
//        shooter = new Shooter(shooterMotor, indexer);
        grabber = new Grabber(uptakeMotor1, uptakeMotor2, grabberServo, touchSensor);

        //odometry = new Odometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);
        simpleOdometry = new SimpleOdometry(drivetrain, gyro, leftOdometer, rightOdometer, backOdometer);

        traj = new TrajectoryCommand(odometry, drivetrain);

        telemetry.addData("Status", "Initialized");

        // stupidSimpleAuto();

        // initVision();

        // if (tfod != null) {
        //     tfod.activate();
        // }

        // while(runtime.seconds() < 3 && trueRec == null) {
        //     trueRec = seeRing();
        // }
        // if (trueRec != null) {
        //     telemetry.addData("Recognized", trueRec.getLabel());
        // } else {
        //     telemetry.addData("Recognized", "None");
        // }
        // telemetry.update();

        waitForStart();

        odometry.update();
        simpleOdometry.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Odometry", "X Value (in): " + odometry.getCurrentVector2D().getX());
        telemetry.addData("Odometry", "Y Value (in): " + odometry.getCurrentVector2D().getY());

        telemetry.addData("Angle", gyro.getAngle());
        telemetry.update();

        if(opModeIsActive()){
            // initVuforia();
            // station1();
            // station2();
            station3();

        }

        runtime.reset();

        // ** MAIN LOOP **//
        while (opModeIsActive()) {
            // if(trueRec == null) {
            //     noRings();
            // }
            // else if(trueRec.getLabel().equals("Single")) {
            //     single();
            // }
            // else {
            //     quad();
            // }


        }

        // if (tfod != null) {
        //     tfod.shutdown();

        // // stupidSimpleAuto();

        // }
    } // end of active op mode


    //** METHODS **//

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // Wait command (accepts time in milliseconds)
    public static void sleepMil(long sleepTime)
    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }
    public void station1() {
        // forwardInches(-0.5, 5);
        double power = -0.5;
        double inches = 1000/0.5;
        drivetrain.drive(power, -0.04, 0, false);
        sleepMil((int)(2600));
        drivetrain.drive(0, 0, 0, false);
        drivetrain.drive(0,-1,0,false);
        sleepMil((int)(3700/3.0));
        drivetrain.drive(0,0,0,false);

    }
    public void station2() {
        // forwardInches(-0.5, 5);
        double power = -0.5;
        double inches = 1000/0.5;
        drivetrain.drive(power, 0, 0, false);
        sleepMil(3500);
        drivetrain.drive(0, 0, 0, false);
    }
    public void station3() {
        double power = -0.5;
        double inches = 1000/0.5;
        drivetrain.drive(power, 0, 0, false);
        sleepMil((int)(2600));
        drivetrain.drive(0, 0, 0, false);
        drivetrain.drive(0,1,0,false);
        sleepMil((int)(3700/3.0));
        drivetrain.drive(0,0,0,false);
    }

    public void forwardInches(double power, double inches) {
        double currentY = simpleOdometry.getCurrentVector2D().getY();
        if(inches > 0) {
            if (simpleOdometry.getCurrentVector2D().getY() < currentY + inches) {
                drivetrain.drive(power, 0, 0, false);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
        else {
            if(simpleOdometry.getCurrentVector2D().getY() > currentY - inches) {
                drivetrain.drive(power, 0, 0, false);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
    }

    public void strafeInches(double power, double inches) {
        double currentY = simpleOdometry.getCurrentVector2D().getY();
        if(inches > 0) {
            if(simpleOdometry.getCurrentVector2D().getY() < currentY + inches) {
                drivetrain.drive(0, power, 0, false);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
        else {
            if(simpleOdometry.getCurrentVector2D().getY() > currentY - inches) {
                drivetrain.drive(0, power, 0, false);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
    }

    public void turnAngle(double power, double angle) {
        double currentAngle = gyro.getAngle();
        if(angle > 0) {
            if(gyro.getAngle() < currentAngle + angle) {
                drivetrain.drive(0, 0, power);
            }
            else {
                drivetrain.drive(0, 0, 0);
            }
        }
        else {
            if(gyro.getAngle() > currentAngle - angle) {
                drivetrain.drive(0, 0, power);
            }
            else{
                drivetrain.drive(0, 0, 0);
            }
        }
    }

    /*public void armDown() {
        while(armMotor.getCurrentPosition() < 1400) {
            grabber.down();
        }
        grabber.off();
    }*/

    /*public void armUp() {
        while(armMotor.getCurrentPosition() > 20) {
            grabber.up();
        }
        grabber.off();
    }*/

    /*public void armSlightlyUp() {
        while(armMotor.getCurrentPosition() > 1000) {
            grabber.up();
        }
        grabber.off();
    }*/

    public void driveWaypoint(Pose2D targetWaypoint) {
        odometry.update();
        traj.setDrivePower(.6);
        traj.setRotatePower(.4);
        while(!traj.atWaypoint(targetWaypoint)) {
            odometry.update();
            traj.goToPoint(targetWaypoint);
        }
        drivetrain.drive(0, 0, 0);
    }



    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // the line above gives an error in android studio but builds fine in onbot java
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    // private void initVuforia() {
    //     VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    //     parameters.vuforiaLicenseKey = VUFORIA_KEY;
    //     parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    //     //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

    //     //  Instantiate the Vuforia engine
    //     vuforia = ClassFactory.getInstance().createVuforia(parameters);

    //     // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    // }


    public void initVision() {
        // Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //TensorFlow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.activate();
        tfod.setZoom(2.5, 1.78);
    }

    public Recognition seeRing() {
        Recognition rec = null;
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                double maxArea = Integer.MIN_VALUE;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getHeight() * recognition.getWidth() > maxArea) {
                        rec = recognition;
                        maxArea = rec.getHeight() * rec.getWidth();
                    }
                }
            }
        }

        return rec;
    }

    private void single() {
        //shooter.setIndexer(.9);
        grabber.closeGrabber();

        driveWaypoint(new Pose2D(0, 13, 0)); // move to the left of the rings
        sleepMil(250);
        driveWaypoint(new Pose2D(79, 13 ,0)); // go to wobble square
        sleepMil(250);
        //armDown();
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(8.9, 13, 0)); // go to the left of the rings
        sleepMil(250);
        driveWaypoint(new Pose2D(8.9, -1.5, 0)); // go to wobble goal
        grabber.closeGrabber();

        sleepMil(350);
        driveWaypoint(new Pose2D(8.9, 13, 0)); // move to the left of the rings
        driveWaypoint(new Pose2D(78, 13 ,0)); // go to wobble square
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(50, 0, 90)); // go to shooter position

        grabber.closeGrabber(); // put arm up
        //armUp();

        //shooter.on(); // start shooter sequence
        sleepMil(750);
        //shooter.setIndexer(0); // back
        sleepMil(500);
        //shooter.setIndexer(.65); // shoot reset
        sleepMil(500);
        //shooter.setIndexer(0); // back
        sleepMil(500);
        //shooter.setIndexer(.65); // shoot reset
        sleepMil(500);
        //shooter.setIndexer(0); // back
        sleepMil(500);
        //shooter.setIndexer(.65); // shoot reset
        sleepMil(500);
        //shooter.setIndexer(0); // back
        sleepMil(500);
        //shooter.setIndexer(.65); // shoot reset
        sleepMil(500);
        //shooter.off();

        driveWaypoint(new Pose2D(60, 0, 90)); // drive up to line

        stop();
    }

    private void quad() {
        //shooter.setIndexer(.9);
        grabber.closeGrabber();

        driveWaypoint(new Pose2D(0, 14, 0)); // move to the left of the rings
        sleepMil(250);
        driveWaypoint(new Pose2D(80, 14 ,0));
        driveWaypoint(new Pose2D(101, -12 , 0)); // go to wobble square
        sleepMil(250);
        //armDown();
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(8.9, 17, 0)); // go to the left of the rings
        sleepMil(250);
        driveWaypoint(new Pose2D(8.9, -1.5, 0)); // go to wobble goal
        grabber.closeGrabber();

        sleepMil(350);
        driveWaypoint(new Pose2D(8.9, 17, 0)); // move to the left of the rings
        driveWaypoint(new Pose2D(80, 17, 0));
        driveWaypoint(new Pose2D(98, -11 , -0)); // go to wobble square
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(48, 0, 90)); // go to shooter position

        grabber.closeGrabber(); // put arm up
        //armUp();

        // //shooter.on(); // start shooter sequence
        // sleepMil(750);
        // //shooter.setIndexer(0); // back
        // sleepMil(500);
        // //shooter.setIndexer(.65); // shoot reset
        // sleepMil(500);
        // shooter.setIndexer(0); // back
        // sleepMil(500);
        // shooter.setIndexer(.65); // shoot reset
        // sleepMil(500);
        // shooter.setIndexer(0); // back
        // sleepMil(500);
        // shooter.setIndexer(.65); // shoot reset
        // sleepMil(500);
        // shooter.setIndexer(0); // back
        // sleepMil(500);
        // shooter.setIndexer(.65); // shoot reset
        // sleepMil(500);
        // shooter.off();

        driveWaypoint(new Pose2D(60, 0, 90)); // drive up to line

        stop();

    }

    private void noRings() {
        //shooter.setIndexer(.9);
        grabber.closeGrabber();

        driveWaypoint(new Pose2D(40, 0, 0)); // move forward
        driveWaypoint(new Pose2D(62, -10 ,0)); // go to wobble square
        sleepMil(250);
        //armDown();
        grabber.openGrabber();
        sleepMil(250);
        driveWaypoint(new Pose2D(8.9, 11, 0)); // go to back wall
        sleepMil(250);
        driveWaypoint(new Pose2D(8.9, -1.6, 0)); // go to wobble goal
        grabber.closeGrabber();

        sleepMil(250);
        driveWaypoint(new Pose2D(59, -9, 0)); // drop off wobble goal
        grabber.openGrabber();
        sleepMil(250);

        driveWaypoint(new Pose2D(49, 0, 90)); // go to shooter position

        grabber.closeGrabber(); // put arm up
        //armUp();

        sleepMil(1000);

        // shooter.on(); // start shooter sequence
        // sleepMil(750);
        // shooter.setIndexer(0); // back
        // sleepMil(500);
        // shooter.setIndexer(.65); // shoot reset
        // sleepMil(500);
        // shooter.setIndexer(0); // back
        // sleepMil(500);
        // shooter.setIndexer(.65); // shoot reset
        // sleepMil(500);
        // shooter.setIndexer(0); // back
        // sleepMil(500);
        // shooter.setIndexer(.65); // shoot reset
        // sleepMil(500);
        // shooter.setIndexer(0); // back
        // sleepMil(500);
        // shooter.setIndexer(.65); // shoot reset
        // sleepMil(500);
        // shooter.off();

        driveWaypoint(new Pose2D(60, 0, 90)); // drive up to line

        stop();

    }

}// end of class
