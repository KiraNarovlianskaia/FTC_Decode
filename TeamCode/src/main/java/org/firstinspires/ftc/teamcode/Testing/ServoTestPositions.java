package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name="Servo Test Pos", group="Robot")
public class ServoTestPositions extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo servoL = hardwareMap.get(Servo.class, "servo_left");
        Servo servoM = hardwareMap.get(Servo.class, "servo_mid");
        Servo servoR = hardwareMap.get(Servo.class, "servo_right");

        servoR.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {


            // left servo
            if (gamepad1.x) {
                servoL.setPosition(0);
                sleep(800);
                servoL.setPosition(1);
            }

            // mid servo
            if (gamepad1.a) {
                servoM.setPosition(0);
                sleep(800);
                servoM.setPosition(1);
            }

            // right servo
            if (gamepad1.b) {
                servoR.setPosition(0);
                sleep(800);
                servoR.setPosition(1);
            }

            // all 3 servos
            if (gamepad1.y) {
                servoL.setPosition(0);
                servoM.setPosition(0);
                servoR.setPosition(0);
                sleep(800);
                servoL.setPosition(1);
                servoM.setPosition(1);
                servoR.setPosition(1);
            }
        }
    }
}
