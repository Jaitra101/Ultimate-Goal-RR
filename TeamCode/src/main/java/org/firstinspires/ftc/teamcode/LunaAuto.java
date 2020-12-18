package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Luna Auto TFOD")
public class LunaAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    SampleMecanumDrive drivetrain;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZT5cbD/////AAABmfJnIeddQEINhStd867KmsE/U5DxLb3la9BlLgqlAj7pwpiD/JFpwc61NP8dil/k9TpMthRa3J0OFg2P1oaCjeRLb8s4ku8mWqY142NCUbQmrnMtzCDezbfhmeXOdgONV7+oW2Nu50zXzUwG/tkR8UgWiMkSU9M3ZEyZEhaG4sscMoY/tW23IWyq6PoMwIz8aQdtc+hm68hvEES4GYTPnxz0XvyPSNcGWmBPMGWYmKRy9i7ZGJG6/L29z3Y7AP2bnQ5gzHPuWWr6plIcxNP2jvaafqTPz8Nn2tjZ7yk3KPeHmf4MLjuseGCWV8wub/+sDKt2B5/vVntltf5/0gVAb25avPL9aNOGLDHEePembGAb";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        drivetrain = new SampleMecanumDrive(hardwareMap);
        initVuforia();
        initTfod();


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        waitForStart();


            if (opModeIsActive())
            {
                if (tfod != null) {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 0) {
                                // empty list.  no objects recognized.
                                telemetry.addData("TFOD", "No items detected.");
                                telemetry.addData("Target Zone", "A");
                                telemetry.update();
                                ZoneAT();
                            } else {
                                // list is not empty.
                                // step through the list of recognitions and display boundary info.
                                int i = 0;
                                for (Recognition recognition : updatedRecognitions) {
                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());

                                    // check label to see which target zone to go after.
                                    if (recognition.getLabel().equals("Single")) {
                                        telemetry.addData("Target Zone", "B");
                                        telemetry.update();
                                        ZoneBT();
                                    } else if (recognition.getLabel().equals("Quad")) {
                                        telemetry.addData("Target Zone", "C");
                                        telemetry.update();
                                        ZoneCT();
                                    } else {
                                        telemetry.addData("Target Zone", "UNKNOWN");
                                        telemetry.update();
                                    }
                                }
                            }
                        }

                }
            }

        if (tfod != null) {
            tfod.shutdown();
        }
    }
    
    public void ZoneAT()
    {
        Trajectory shootPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(50,6, Math.toRadians(9)))
                .build();
        drivetrain.followTrajectory(shootPosition);

        Trajectory wobbleRecenter = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 16, Math.toRadians(5)))
                .build();
        drivetrain.followTrajectory(wobbleRecenter);

        Trajectory deliverWobble = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(70, -9, Math.toRadians(9)))
                .build();
        drivetrain.followTrajectory(deliverWobble);
        wobbleDrop();

        startFlywheel(1.00);
        Trajectory backPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-32.25, -3, Math.toRadians(-4.5)))
                .build();
        drivetrain.followTrajectory(backPosition);
        ringIndex();
        stopFlywheel();

        Trajectory frontPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(30, -3, Math.toRadians(6)))
                .build();
        drivetrain.followTrajectory(frontPosition);
    }
    public void ZoneBT()
    {
        startFlywheel(1.00);

        Trajectory shootPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(9)))
                .build();
        drivetrain.followTrajectory(shootPosition);

        ringIndex();
        stopFlywheel();

        Trajectory wobbleRecenter = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 24, Math.toRadians(9)))
                .build();
        drivetrain.followTrajectory(wobbleRecenter);

        Trajectory deliverWobble = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(144, -72, Math.toRadians(9)))
                .build();
        drivetrain.followTrajectory(deliverWobble);

        wobbleDrop();
        Trajectory linePark = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-48, 0, Math.toRadians(9)))
                .build();
        drivetrain.followTrajectory(linePark);
    }
    public void ZoneCT()
    {
        Trajectory shootPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(50,6, Math.toRadians(9)))
                .build();
        drivetrain.followTrajectory(shootPosition);

        Trajectory wobbleRecenter = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 16, Math.toRadians(5)))
                .build();
        drivetrain.followTrajectory(wobbleRecenter);

        Trajectory deliverWobble = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(130, -9, Math.toRadians(9)))
                .build();
        drivetrain.followTrajectory(deliverWobble);
        wobbleDrop();

        startFlywheel(1.00);
        Trajectory backPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-92.25, -3, Math.toRadians(-4.5)))
                .build();
        drivetrain.followTrajectory(backPosition);
        ringIndex();
        stopFlywheel();

        Trajectory frontPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(32, -3, Math.toRadians(6)))
                .build();
        drivetrain.followTrajectory(frontPosition);
    }
    public void ringIndex() {


        drivetrain.indexer.setPosition(0.65);
        sleep(600);
        drivetrain.indexer.setPosition(0.90);
        startFlywheel(1.0);
        sleep(1500);
        drivetrain.indexer.setPosition(0.65);
        sleep(600);
        drivetrain.indexer.setPosition(0.90);
       startFlywheel(1.0);
        sleep(1500);
        drivetrain.indexer.setPosition(0.65);
        sleep(600);
    }
    public void wobbleDrop() {


        drivetrain.arm.setPower(0.75);
        sleep(300);
        drivetrain.arm.setPower(0.00);
        sleep(900);
        drivetrain.grabber.setPosition(0.30);
        sleep(600);
    }
    public void startFlywheel(double power) {

        drivetrain.flywheel.setPower(power);
        sleep(600);
        if(power > 0) {
            double currentPower = drivetrain.flywheel.getPower();
            double error = (power - currentPower);
            drivetrain.flywheel.setPower(currentPower + error);
        }

    }
    public void stopFlywheel() {
        drivetrain.flywheel.setPower(0.00);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
