package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class LonkusAuto extends LinearOpMode {
    private DcMotorEx extendo;
    private Servo claw;
    private Servo wubbo;
    @Override
    public void runOpMode() throws InterruptedException {
        wubbo = hardwareMap.get(Servo.class, "wubbo");
        claw = hardwareMap.get(Servo.class, "claw");
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        wubbo.setPosition(0.4);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(36)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .back(18)
                .build();
        waitForStart();

        if (isStopRequested()) return;


        wubbo.setPosition(.4);
        drive.followTrajectory(trajectory1);
        //drive.followTrajectory(trajectory2);
        extendo.setTargetPosition(1000);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}