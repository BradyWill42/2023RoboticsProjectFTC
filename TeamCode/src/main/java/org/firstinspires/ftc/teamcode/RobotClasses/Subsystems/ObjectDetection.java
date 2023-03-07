package org.firstinspires.ftc.teamcode.RobotClasses.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * Helper class that provides Object Detection of the Default Signal sleeve or Custom Signal sleeves.
 *
 * Note: For this to operate correctly, you'll need to ensure the Webcam is plugged into USB Port 1
 * and named "Webcam 1" in the config. If you don't see a webcam listed in config after plugging in,
 * press "Scan" in the configuration menu of your driver station app.
 *
 * If you do not provide a custom Tensorflow lite model, this class uses the default PowerPlay model
 * to detect the signal sleeve.
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
 * 6. Your custom model is now available to use!
 */
public class ObjectDetection {
    /// Required properties that must be set.

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    /// The minimum threshold before an object is considered "seen". Adjust this as needed.
    public double minimumConfidenceThreshold = 0.75; // 75%.

    /// Webcam for detecting objects. In your robot configuration, it's expected
    /// to be named "Webcam 1" in the robot configuration.
    private WebcamName webcamName;
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "Acy/Mw7/////AAABmVeoJmayA0LFsPI//25wXiAgKvF8A1IMKKPVnU/YmD2SLqIfVxja/iMw9FGXlbh6ipPCe1BFslVFQra+jdueadfzzLqYiH9I9BAz0gOjuhBQB3bxgRnHI4nFWwaaRd0BYN0MYlgXZCcLjL5YdP/dnk3ffrlMlf4U5IK/yJpsxw6Eum84uoiFq7gnyOAcdJR2zXpF86/e/L0iPQrloOtqspc5Pp4u1ra2e6Sa3fWhHbB2g4wC4nYgi980JBGxZdvL1tkVoMqmbvrylRwF4Jsm7NsDKLjkZRnGULl/xXSFJ9ry45RAEGxh745HQRiJ2/lNpdabZpXONPi5xMscczlGvUpgCNebeumlqXrA7swmEsGE";

    /// Values for configuration.
    private String customModelFilename;
    private String[] customLabels;
    private String lastLabelSeen;

    /// The default model being used. PowerPlay comes pre-installed.
    /// This model is used if no custom model is specified.
    private static final String DEFAULT_MODEL_ASSET = "SignalSleeveDetectionIMPROVED.tflite";
    /// Default labels to look for.
    private static final String[] DEFAULT_LABELS = {
            "square",
            "circle",
            "triangle"
    };

    ///----------------------- Public Functions ---------------------///

    /// Constructor for the class.
    /// hardwareMap - The OpMode HardwareMap.
    /// telemetry - Telemetry for logging.
    public ObjectDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initializeDetection();
    }

    /// Sets the custom model to be used. This should be called after initializing the class.
    public void setCustomModel(String fileName, String[] labels) {
        this.customModelFilename = fileName;
        this.customLabels = labels;
        if (tfod != null) {
            // Use loadModelFromFile() if you have downloaded a custom team model to the
            // Robot Controller's FLASH.
            tfod.loadModelFromFile(customModelFilename, customLabels);
            telemetry.addData("Object Detection", "Custom Model Loaded.");
        }
    }

    /// Activate object detection and set zoom. This should be called prior to start button being pressed.
    /// Once activated, it starts recognizing objects.
    public void activateIfNeeded() {
        if (tfod != null) {
            if (customModelFilename == null) {
                // Load the default model since we didn't set a custom model.
                // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
                tfod.loadModelFromAsset(DEFAULT_MODEL_ASSET, DEFAULT_LABELS);
                telemetry.addData("Object Detection", "Default Model Loaded.");
            }
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.25, 4.0/3.0); // (Our Logitech Webcams are 4:3)

            telemetry.addData("Object Detection", "Activated");
        }
    }

    /// Should be called once the stop button has been pressed to deactivate tensorflow detection.
    public void deactivateIfNeeded() {
        if (tfod != null) {
            tfod.deactivate();

            telemetry.addData("Object Detection", "Deactivated");
        }
    }

    /// Detect objects. The function will return the name of the last object seen meeting
    /// the minimum threshold provided in setup. If no objects were found, it will return null.
    /// This function should be called multiple times until a value is returned (or not).
    public String detectObjectAndGetLabel() {
        // Setup the tensorflow image recognition.
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // Step through the list of recognitions.
                for (Recognition recognition : updatedRecognitions) {
                    float confidence = recognition.getConfidence();
                    // Log out what is seen.
                    telemetry.addData("Seen Object", "%s (%.0f %% Conf.)", recognition.getLabel(), confidence * 100);
                    // Determine if we found something with our minimum threshold and save.
                    if (confidence >= minimumConfidenceThreshold) {
                        telemetry.addData(""," "); // Adds a line break in the log.
                        lastLabelSeen = recognition.getLabel();
                        telemetry.addData("Confirmed Object", "%s (%.0f %% Conf.)", recognition.getLabel(), confidence * 100);
                    }
                }
            } else {
                telemetry.addData("# Objects Detected", 0);
            }
            return lastLabelSeen;
        }

        return null;
    }

    /// Gets the last object seen, null if nothing was found meeting the minimum threshold.
    public String getLastObjectSeen() {
        return lastLabelSeen;
    }

    ///----------------------- Private Functions ---------------------///

    /// Function that initializes tensorflow object detection. This should be called before the start
    /// button is pressed.
    private void initializeDetection() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Init Vuforia.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Init Tensorflow.
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        // Set the minimum result confidence to low so we get more data. We'll then filter it
        // in this class later on. This helps with testing.
        tfodParameters.minResultConfidence = 0.4f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        telemetry.addData("Object Detection", "Initialized.");
    }
}