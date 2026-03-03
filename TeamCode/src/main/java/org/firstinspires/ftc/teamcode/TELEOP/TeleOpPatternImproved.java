package org.firstinspires.ftc.teamcode.TELEOP;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name="TeleOpPatternImproved")
/*
 This is the final version of the TeleOp. Any new features are implemented here.
 Controls:
   gamepad1:
     - left stick up/down  =  forward/backward
     - right stick left/right  =  rotate
     - left/right triggers  =  side
     - x, a, b  =  push left, mid, right ball respectively (separate)
     - y  =  push all 3 balls

   gamepad2:
     - left stick up/down  =  intake in/out
     - right stick up  =  shoot
     - x, a, b  =  select pattern (x = gpp, a = pgp, b = ppg)
     - y  =  shoot by pattern
     - dpad (circle thingy with arrows)  =  Shooter selection
         > left: Left shooter
         > down: Mid shooter
         > right: Right shooter
         > up: All 3 shooters

*/

public class TeleOpPatternImproved extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor intake;
    DcMotor shooterL, shooterM, shooterR;

    Servo servoL;
    Servo servoM;
    Servo servoR;

    static final double servoPush = 0;
    static final double servoOpen = 1;

    static final double SPEEDFACTOR = 0.55;
    static final double SPEEDROTATE = 0.35;
    static double shootingSpeed = 0.85;

    String[] shootersToPower = {"L", "M", "R"}; // Shooter mode
    String[] PATTERN = {"Purple,", "Purple", "Green"};

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
        rightBack.setDirection(DcMotor.Direction.FORWARD); // this looks weird but trust me it works, just don't change it

        shooterM.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoOpen);

        while (opModeIsActive()) {

            // Push left ball
            if (gamepad1.x) {
                servoL.setPosition(servoPush);
                sleep(500);
                servoL.setPosition(servoOpen);
            }

            // Push middle ball
            if (gamepad1.a) {
                servoM.setPosition(servoPush);
                sleep(500);
                servoM.setPosition(servoOpen);
            }

            // Push right ball
            if (gamepad1.b) {
                servoR.setPosition(servoPush);
                sleep(500);
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
                side = gamepad1.left_trigger - gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0) {
                side = gamepad1.left_trigger;
            } else if (gamepad1.right_trigger > 0) {
                side = -gamepad1.right_trigger;
            }

            // Select pattern
            if (gamepad2.xWasPressed()) {
                PATTERN = new String[]{"Green", "Purple", "Purple"};
            } else if (gamepad2.aWasPressed()) {
                PATTERN = new String[]{"Purple", "Green", "Purple"};
            } else if (gamepad2.bWasPressed()) {
                PATTERN = new String[]{"Purple", "Purple", "Green"};
            }

            // Shoot by pattern
            if (gamepad2.yWasPressed()) {
                shootByPattern(left_ball, mid_ball, right_ball);
            }


            // Set motor movement speeds
            leftFront.setPower((forward * SPEEDFACTOR) - (rotation * SPEEDROTATE) - (side * SPEEDFACTOR));
            leftBack.setPower((forward * SPEEDFACTOR) - (rotation * SPEEDROTATE) + (side * SPEEDFACTOR));
            rightFront.setPower((forward * SPEEDFACTOR) + (rotation * SPEEDROTATE) + (side * SPEEDFACTOR));
            rightBack.setPower((forward * SPEEDFACTOR) + (rotation * SPEEDROTATE) - (side * SPEEDFACTOR));

            intake.setPower(intakeSpeed);
            shooterL.setPower(powerL * shootingSpeed);
            shooterM.setPower(powerM * shootingSpeed);
            shooterR.setPower(powerR * shootingSpeed);


            //----ARTIFACT COLOR DETECTION LOGIC----

            // Normalize colors
            NormalizedRGBA colors_left = colorSensorL.getNormalizedColors();
            NormalizedRGBA colors_mid = colorSensorM.getNormalizedColors();
            NormalizedRGBA colors_right = colorSensorR.getNormalizedColors();

            // Left ball
            if (colors_left.alpha > colors_left.blue && colors_left.alpha > colors_left.green) {
                left_ball = "No Color";
            } else if (colors_left.blue > colors_left.green) {
                left_ball = "Purple";
            } else {
                left_ball = "Green";
            }

            // Mid ball
            if (colors_mid.alpha > colors_mid.blue && colors_mid.alpha > colors_mid.green) {
                mid_ball = "No Color";
            } else if (colors_mid.blue > colors_mid.green) {
                mid_ball = "Purple";
            } else {
                mid_ball = "Green";
            }

            // Right ball
            if (colors_right.alpha > colors_right.blue && colors_right.alpha > colors_right.green) {
                right_ball = "No Color";
            } else if (colors_right.blue > colors_right.green) {
                right_ball = "Purple";
            } else {
                right_ball = "Green";
            }

        }
    }

    public void shootByPattern(String left, String middle, String right) {

        // Map balls and servos to arrays
        String[] balls = {left, middle, right};
        Servo[] servos = {servoL, servoM, servoR};

        // Keep track of which balls have been shot
        boolean[] shot = {false, false, false};

        // Activate all shooters
        shooterL.setPower(shootingSpeed);
        shooterM.setPower(shootingSpeed);
        shooterR.setPower(shootingSpeed);
        sleep(3000);

        // Loop through motif pattern
        for (String s : PATTERN) {
            // Find the ball that matches the current motif color
            for (int j = 0; j < balls.length; j++) {
                if (!shot[j] && balls[j].equals(s)) {
                    servos[j].setPosition(servoPush);
                    sleep(800);
                    servos[j].setPosition(servoOpen);
                    shot[j] = true;
                    break; // Move to next motif color
                }
            }
        }
    }
}    