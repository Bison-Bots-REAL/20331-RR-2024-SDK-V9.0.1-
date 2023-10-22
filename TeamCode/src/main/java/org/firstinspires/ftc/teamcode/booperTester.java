package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class booperTester extends LinearOpMode {
    private DcMotorEx spinny;

    private Servo booper;
    private float speed = (float) 0;

    public float ticksperrev = (float) 537.6;

    public float maxrpm = (float) 312;

    @Override
    public void runOpMode() {
        spinny = hardwareMap.get(DcMotorEx.class, "spinny");

        booper = hardwareMap.get(Servo.class, "booper");

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

            if (gamepad1.dpad_down) {
                booper.setPosition(0.2); //Lonkus starting with 0.2
            }
            else if (gamepad1.dpad_left) {
                booper.setPosition(0.3); //Lonkus starting with 0.3
            }
            else if (gamepad1.dpad_right) {
                booper.setPosition(0.4); //Lonkus starting with 0.4
            }
            else if (gamepad1.dpad_up) {
                booper.setPosition(0.5); //Lonkus starting with 0.5
            }

            spinny.setVelocity(speed * (2760));


            telemetry.addData("Wheel Velocity", (Math.round((spinny.getVelocity() / 2760) * 100)) + "%");
            telemetry.addData("Launch Speed", (((spinny.getVelocity() / 537.6) * 7238.23)/10000) + "m/s");
            telemetry.update();
        }
    }
}
