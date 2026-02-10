package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name = "TwoServoTeleOp")
public class TwoServoTeleOp extends LinearOpMode {

    private Servo servoA;
    private Servo servoB;

    @Override
    public void runOpMode() {

        servoA = hardwareMap.servo.get("servoA");
        servoB = hardwareMap.servo.get("servoB");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                moveServoToAngle(servoA, 0);
            }

            if (gamepad1.b) {
                moveServoToAngle(servoB, 0);
            }

            telemetry.addData("ServoA Position", servoA.getPosition());
            telemetry.addData("ServoB Position", servoB.getPosition());
            telemetry.update();
        }
    }

    private void moveServoToAngle(Servo servo, double angle) {
        double position = angle / 270.0;
        servo.setPosition(position);
    }
}