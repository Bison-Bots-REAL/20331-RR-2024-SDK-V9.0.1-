package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp
public class LonkusLauncher extends LinearOpMode {
    private DcMotorEx spinny;
    private float speed = (float) 0;

    public float ticksperrev = (float) 537.6;

    public float maxrpm = (float) 312;

    @Override
    public void runOpMode() {
        spinny = hardwareMap.get(DcMotorEx.class, "spinny");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (-gamepad1.right_stick_y > 0) {
                if (speed <= 1) {
                    speed += 0.01*-gamepad1.right_stick_y;
                }
            }
            else if (-gamepad1.right_stick_y < 0) {
                if (speed >= 0) {
                    speed -= 0.01*gamepad1.right_stick_y;
                }
            }

            spinny.setVelocity(speed * (2760));


            telemetry.addData("Wheel Velocity", (Math.round((spinny.getVelocity() / 2760) * 100)) + "%");
            telemetry.addData("Launch Speed", (((spinny.getVelocity() / 537.6) * 7238.23)/10000) + "m/s");
            telemetry.update();
        }
    }
}
