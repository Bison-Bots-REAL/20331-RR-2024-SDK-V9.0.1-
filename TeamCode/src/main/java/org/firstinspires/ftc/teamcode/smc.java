package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
public class smc extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    //LonkusLauncherMath variables;
    private DcMotorEx spinny;




    private static float DroneWeight = (float) 0.0045; //should be the weight of a typical printer paper (IN KILOGRAMS) according to online is about 4.5 grams

    //both related to Y axis movement

    private static float StaticGravityAccel = (float) 9.81; //gravity acceleration  = 9.81 m/s^2

    private static float Gravity = (DroneWeight*StaticGravityAccel); //downward force (IN NEWTONS)
    private static float VerticalDrag; //resistance to loss in altitude (IN NEWTONS)

    //both related to X axis movement

    private static float AppliedForce; //Forward force (IN NEWTONS) (DroneWeight*(((spinny.getVelocity() / 537)*7238.23)/1000)) approx meters per second
    private static float HorizontalDrag; //Resistance to forward force (IN NEWTONS)

    private float LandingLocation; //approximated final crash location





    public static double b2d(boolean b) {
        if (b) {
            return 1;
        }
        return 0;
    }
    public void bdrive() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            (-gamepad1.left_stick_y + b2d(gamepad1.dpad_up) - b2d(gamepad1.dpad_down)),
                            (-gamepad1.left_stick_x + b2d(gamepad1.dpad_left) - b2d(gamepad1.dpad_right)),
                            (-gamepad1.right_stick_x + gamepad1.left_trigger - gamepad1.right_trigger)
                    )
            );
        }
    }
    public void april() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        initAprilTag();
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            targetFound = false;
            // Used to hold the data for a detected AprilTag
            AprilTagDetection desiredTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                }
            }
            if (targetFound) {
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            }
        }
        telemetry.update();
        sleep(10);
    }
    public void setManualExposure(int exposureMS, int gain){
        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    public void initAprilTag () {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    public float LonkusLauncherMath(float velocity) {

        spinny = hardwareMap.get(DcMotorEx.class, "spinny");

        // had to pause for now: private float FinalVelocity = ;

        //AppliedForce = FinalVelocity*DroneWeight;







        return LandingLocation;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}