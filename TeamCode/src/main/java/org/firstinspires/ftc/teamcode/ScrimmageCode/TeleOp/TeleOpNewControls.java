package org.firstinspires.ftc.teamcode.ScrimmageCode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
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

        String left_ball = "No Color";
        String mid_ball = "No Color";
        String right_ball = "No Color";

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
        shooterM = hardwareMap.get(DcMotor.class, "shooter_mid");
        shooterR = hardwareMap.get(DcMotor.class, "shooter_right");

        servoL = hardwareMap.get(Servo.class, "servo_left");
        servoM = hardwareMap.get(Servo.class, "servo_mid");
        servoR = hardwareMap.get(Servo.class, "servo_right");

        NormalizedColorSensor colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "ball_color_left");
        NormalizedColorSensor colorSensorM = hardwareMap.get(NormalizedColorSensor.class, "ball_color_mid");
        NormalizedColorSensor colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "ball_color_right");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

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
            intakeSpeed = -gamepad2.left_stick_y;
            shooterStick = -gamepad2.right_stick_y;

            // Activate specific shooters based on chosen mode
            double powerL = 0;
            double powerM = 0;
            double powerR = 0;

            for (String s : shootersToPower) {
                if (s.equals("L")) powerL = shooterStick;
                if (s.equals("M")) powerM = -shooterStick;
                if (s.equals("R")) powerR = -shooterStick;
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


            // Normalize the colors
            NormalizedRGBA colors_left = colorSensorL.getNormalizedColors();
            NormalizedRGBA colors_mid = colorSensorM.getNormalizedColors();
            NormalizedRGBA colors_right = colorSensorR.getNormalizedColors();

            // Ball 1
            if (colors_left.alpha > colors_left.blue && colors_left.alpha > colors_left.green) {
                right_ball = "No Color";
            } else if (colors_left.blue > colors_left.green) {
                right_ball = "Purple";
            } else {
                right_ball = "Green";
            }

            // Ball 2
            if (colors_mid.alpha > colors_mid.blue && colors_mid.alpha > colors_mid.green) {
                mid_ball = "No Color";
            } else if (colors_mid.blue > colors_mid.green) {
                mid_ball = "Purple";
            } else {
                mid_ball = "Green";
            }

            if (colors_right.alpha > colors_right.blue && colors_right.alpha > colors_right.green) {
                right_ball = "No Color";
            } else if (colors_right.blue > colors_right.green) {
                right_ball = "Purple";
            } else {
                right_ball = "Green";
            }
            
        }
    }
}    