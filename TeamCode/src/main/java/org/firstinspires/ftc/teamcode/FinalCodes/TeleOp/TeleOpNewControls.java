package org.firstinspires.ftc.teamcode.FinalCodes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name="TeleOpNewControls")

public class TeleOpNewControls extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor intake;
    DcMotor shooterL, shooterM, shooterR;

    Servo servoL;
    Servo servoM;
    Servo servoR;

    static final double servoPush = 0.45;
    static final double servoOpen = 0;
    static final double servoReady = 0.18;

    static final double SPEEDFACTOR = 0.55;
    static double shootingSpeed = 0.85;

    String ball_left_color = "None";
    String ball_middle_color = "None";
    String ball_right_color = "None";

    String[] shootersToPower = {"L", "M", "R"};

    public void runOpMode() {

        double forward;
        double rotation;
        double side = 0;
        double intakeSpeed;
        double shooterStick;

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotor.class, "shooter_left");
        shooterM = hardwareMap.get(DcMotor.class, "shooter_middle");
        shooterR = hardwareMap.get(DcMotor.class, "shooter_right");

        servoL = hardwareMap.get(Servo.class, "servo_left");
        servoM = hardwareMap.get(Servo.class, "servo_middle");
        servoR = hardwareMap.get(Servo.class, "servo_right");

        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ball_color");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "ball_color2");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "ball_color3");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoOpen);

        while (opModeIsActive()) {

            side = 0;

            // Push left ball
            if (gamepad1.x) {
                servoL.setPosition(servoPush);
                sleep(400);
                servoL.setPosition(servoOpen);
            }

            // Push middle ball
            if (gamepad1.a) {
                servoM.setPosition(servoPush);
                sleep(400);
                servoM.setPosition(servoOpen);
            }

            // Push right ball
            if (gamepad1.b) {
                servoR.setPosition(servoPush);
                sleep(400);
                servoR.setPosition(servoOpen);
            }

            // Push all 3 balls
            if (gamepad1.y) {
                servoL.setPosition(servoPush);
                servoR.setPosition(servoPush);
                sleep(400);
                servoL.setPosition(servoOpen);
                servoR.setPosition(servoOpen);
            }

            // Activate different shooters based on which dpad button was pressed
            if (gamepad2.dpad_left) {
                shootersToPower = new String[]{"L"};
            } else if (gamepad2.dpad_down) {
                shootersToPower = new String[]{"M"};
            } else if (gamepad2.dpad_right) {
                shootersToPower = new String[]{"R"};
            } else if (gamepad2.dpad_up) {
                shootersToPower = new String[]{"L", "M", "R"};
            }

            // Bind motors to gamepad events
            forward = gamepad1.left_stick_y;
            rotation = gamepad1.right_stick_x;
            intakeSpeed = gamepad2.left_stick_y;
            shooterStick = -gamepad2.right_stick_y;

            // Activate specific shooters based on chosen mode
            double powerL = 0;
            double powerM = 0;
            double powerR = 0;

            for (String s : shootersToPower) {
                if (s.equals("L")) powerL = shooterStick;
                if (s.equals("M")) powerM = shooterStick;
                if (s.equals("R")) powerR = shooterStick;
            }

            // Left trigger moves robot left, right trigger is right
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                side = gamepad1.right_trigger - gamepad1.left_trigger;
            } else if (gamepad1.left_trigger > 0) {
                side = -gamepad1.left_trigger;
            } else if (gamepad1.right_trigger > 0) {
                side = gamepad1.right_trigger;
            }

            leftFront.setPower((forward - rotation - side) * SPEEDFACTOR);
            leftBack.setPower((forward - rotation + side) * SPEEDFACTOR);
            rightFront.setPower((forward + rotation + side) * SPEEDFACTOR);
            rightBack.setPower((forward + rotation - side) * SPEEDFACTOR);

            intake.setPower(intakeSpeed);
            shooterL.setPower(powerL * shootingSpeed);
            shooterM.setPower(powerM * shootingSpeed);
            shooterR.setPower(powerR * shootingSpeed);
        }
    }
}