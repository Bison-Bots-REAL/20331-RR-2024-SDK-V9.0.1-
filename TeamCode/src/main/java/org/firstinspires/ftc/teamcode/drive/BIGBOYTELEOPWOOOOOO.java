package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.smc;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class BIGBOYTELEOPWOOOOOO extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected
    private DcMotorEx spinny;

    private DcMotor topleft;
    private DcMotor bottomleft;
    private DcMotor topright;
    private DcMotor bottomright;
    private DcMotorEx extendo;
    private Servo claw;
    private Servo wubbo;
    private DcMotorEx leftarm;
    private DcMotorEx rightarm;
    int pos = 0;
    DigitalChannel redfuny;
    DigitalChannel greenfuny;

    @Override
    public void runOpMode() {
        redfuny = hardwareMap.get(DigitalChannel.class, "red");
        greenfuny = hardwareMap.get(DigitalChannel.class, "gren");
        redfuny.setMode(DigitalChannel.Mode.OUTPUT);
        greenfuny.setMode(DigitalChannel.Mode.OUTPUT);
        spinny = hardwareMap.get(DcMotorEx.class, "spinny");
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        topleft = hardwareMap.get(DcMotorEx.class, "topleft");
        bottomleft = hardwareMap.get(DcMotorEx.class, "bottomleft");
        topright = hardwareMap.get(DcMotorEx.class, "topright");
        bottomright = hardwareMap.get(DcMotorEx.class, "bottomright");
        claw = hardwareMap.get(Servo.class, "claw");
        wubbo = hardwareMap.get(Servo.class, "wubbo");
        leftarm = hardwareMap.get(DcMotorEx.class, "leftarm");
        rightarm = hardwareMap.get(DcMotorEx.class, "rightarm");
        redfuny.setState(false);
        greenfuny.setState(true);
        initAprilTag();
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        topleft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomleft.setDirection(DcMotorSimple.Direction.REVERSE);

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for  a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null)
                        && ((DESIRED_TAG_ID >= 0) || (detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            }
            else {
                telemetry.addData(">", "Drive using joystick to find target\n");
            }
            while (!isStopRequested()) {
                april();

                //COMMENTTED OUT BY LONKUS

                drive.setWeightedDrivePower(
                        new Pose2d(
                                (-gamepad1.left_stick_y + smc.b2d(gamepad1.dpad_up) - smc.b2d(gamepad1.dpad_down)),
                                (-gamepad1.left_stick_x + smc.b2d(gamepad1.dpad_left) - smc.b2d(gamepad1.dpad_right)),
                                (-gamepad1.right_stick_x + gamepad1.left_trigger - gamepad1.right_trigger)
                        )
                );
                drive.update();
                /*
                topleft.setPower((-gamepad1.left_stick_y)-gamepad1.left_trigger+gamepad1.right_trigger);
                bottomleft.setPower((-gamepad1.left_stick_y)+gamepad1.left_trigger-gamepad1.right_trigger);
                topright.setPower((-gamepad1.right_stick_y)+gamepad1.left_trigger-gamepad1.right_trigger);
                bottomright.setPower((-gamepad1.right_stick_y)-gamepad1.left_trigger+gamepad1.right_trigger);
                */
                if (gamepad1.right_trigger > 0) {
                    leftarm.setPower(1);
                    rightarm.setPower(1);
                }
                else if (gamepad1.left_trigger > 0) {
                    leftarm.setPower(-1);
                    rightarm.setPower(-1);
                }
                else {
                    leftarm.setPower(0);
                    rightarm.setPower(0);
                }
                /*
                if (gamepad2.left > 0) {
                    spinny.setVelocity(gamepad1.right_trigger * (2760));
                }
                else {
                    spinny.setVelocity(0);
                }
                 */
                if (gamepad2.left_stick_y > 0 && (extendo.getCurrentPosition() > 280) || gamepad2.guide) { //lonkus changed to 280 (original was 170)
                    extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendo.setPower(-gamepad2.left_stick_y);
                    pos = extendo.getCurrentPosition();
                }
                else if (gamepad2.left_stick_y < 0 && (extendo.getCurrentPosition() < 4200) || gamepad2.guide) {
                    extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendo.setPower(-gamepad2.left_stick_y);
                    pos = extendo.getCurrentPosition();
                }
                else {
                    extendo.setTargetPosition(pos);
                    extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad2.right_trigger > 0) {
                    claw.setPosition(0);
                }
                else if (gamepad2.left_trigger > 0) {
                    claw.setPosition(1f);
                }
                if (gamepad2.dpad_down) {
                    wubbo.setPosition(0.05); //Lonkus changed to 0.03 (original was 0)
                }
                else if (gamepad2.dpad_up){
                    wubbo.setPosition(.4);
                }
                else if (gamepad2.dpad_right){
                    wubbo.setPosition(.3);
                }
                else if (gamepad2.dpad_left){
                    wubbo.setPosition(.25);
                }

                //CONTROLS ADDED BY LONKUS

                if (gamepad1.x) {
                    extendo.setTargetPosition(300);
                    extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (gamepad1.a){
                    extendo.setTargetPosition(1450);
                    extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (gamepad1.b){
                    extendo.setTargetPosition(2350);
                    extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (gamepad1.y){
                    extendo.setTargetPosition(3300);
                    extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                //END OF CONTROLS ADDED BY LONKUS


                telemetry.addData("Encoder Position", (extendo.getCurrentPosition()));
                telemetry.addData("Vel", ((spinny.getVelocity() / 2760) * 100) + "%");
                telemetry.update();
            }
        }
    }
    public void april() {
            targetFound = false;
            // Used to hold the data for a detected AprilTag
            AprilTagDetection desiredTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    redfuny.setState(true);
                    greenfuny.setState(false);
                    desiredTag = detection;
                    break;
                }
            }
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
}
