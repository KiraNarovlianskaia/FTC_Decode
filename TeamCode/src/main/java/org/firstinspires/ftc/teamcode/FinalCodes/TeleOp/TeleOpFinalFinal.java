package org.firstinspires.ftc.teamcode.FinalCodes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name="TeleOpFinal")

public class TeleOpFinalFinal extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor intake, shooter;
    Servo servoL, servoR;

    // Servo positions
    static final double servoPush  = 0.45;
    static final double servoOpen  = 0.0;
    static final double servoReady = 0.18;

    // Drive & mechanism speeds
    static final double SPEEDFACTOR = 0.55;
    static final double intakeSpeed = 0.9;
    static double shootingSpeed = 0.85;

    // Color sensor thresholds
    static final float BALL_PRESENT_THRESHOLD = 0.03f;
    static final float COLOR_MARGIN = 0.02f;

    // Ball state
    boolean ball_left_present  = false;
    boolean ball_right_present = false;
    String leftBallColor  = "none";
    String rightBallColor = "none";

    @Override
    public void runOpMode() {

        // Drive motors
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack   = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack  = hardwareMap.get(DcMotor.class, "right_back");

        intake  = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        servoL = hardwareMap.get(Servo.class, "left_servo");
        servoR = hardwareMap.get(Servo.class, "right_servo");

        NormalizedColorSensor colorSensorLeft  =
                hardwareMap.get(NormalizedColorSensor.class, "ball_color");
        NormalizedColorSensor colorSensorRight =
                hardwareMap.get(NormalizedColorSensor.class, "ball_color2");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        servoL.setPosition(servoOpen);
        servoR.setPosition(servoOpen);

        while (opModeIsActive()) {

            /* ---------------- SERVO MANUAL CONTROL ---------------- */

            if (gamepad2.xWasPressed()) {
                servoL.setPosition(servoPush);
                sleep(400);
                servoL.setPosition(servoOpen);
            }

            if (gamepad2.bWasPressed()) {
                servoR.setPosition(servoPush);
                sleep(400);
                servoR.setPosition(servoOpen);
            }

            if (gamepad2.yWasPressed()) {
                servoL.setPosition(servoPush);
                servoR.setPosition(servoPush);
                sleep(400);
                servoL.setPosition(servoOpen);
                servoR.setPosition(servoOpen);
            }

            /* ---------------- DRIVE CONTROL ---------------- */

            double forward  = gamepad1.left_stick_y;
            double side     = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            leftFront.setPower((forward - rotation - side) * SPEEDFACTOR);
            leftBack.setPower((forward - rotation + side) * SPEEDFACTOR);
            rightFront.setPower((forward + rotation + side) * SPEEDFACTOR);
            rightBack.setPower((forward + rotation - side) * SPEEDFACTOR);

            /* ---------------- INTAKE & SHOOTER ---------------- */

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            shooter.setPower(gamepad2.left_stick_y * shootingSpeed);

            /* ---------------- COLOR SENSOR LOGIC ---------------- */

            NormalizedRGBA leftColors  = colorSensorLeft.getNormalizedColors();
            NormalizedRGBA rightColors = colorSensorRight.getNormalizedColors();

            // LEFT BALL
            if (leftColors.alpha < BALL_PRESENT_THRESHOLD) {
                ball_left_present = false;
                leftBallColor = "none";
            } else {
                ball_left_present = true;
                if (leftColors.blue > leftColors.green + COLOR_MARGIN) {
                    leftBallColor = "purple";
                } else {
                    leftBallColor = "green";
                }
            }

            // RIGHT BALL
            if (rightColors.alpha < BALL_PRESENT_THRESHOLD) {
                ball_right_present = false;
                rightBallColor = "none";
            } else {
                ball_right_present = true;
                if (rightColors.blue > rightColors.green + COLOR_MARGIN) {
                    rightBallColor = "purple";
                } else {
                    rightBallColor = "green";
                }
            }

            /* ---------------- AUTO SERVO READY ---------------- */

            if (ball_left_present) {
                servoR.setPosition(servoReady);
            }
            if (ball_right_present) {
                servoL.setPosition(servoReady);
            }

            /* ---------------- SHOOTER SPEED TUNING ---------------- */

            if (gamepad1.left_bumper) {
                shootingSpeed -= 0.01;
                sleep(50);
            }

            if (gamepad1.right_bumper) {
                shootingSpeed += 0.01;
                sleep(50);
            }

            /* ---------------- TELEMETRY ---------------- */

            telemetry.addData("Left Ball Present", ball_left_present);
            telemetry.addData("Left Ball Color", leftBallColor);
            telemetry.addData("Left Alpha", leftColors.alpha);

            telemetry.addData("Right Ball Present", ball_right_present);
            telemetry.addData("Right Ball Color", rightBallColor);
            telemetry.addData("Right Alpha", rightColors.alpha);

            telemetry.addData("Shooter Speed", shootingSpeed);
            telemetry.update();

            sleep(50);
        }
    }
}
