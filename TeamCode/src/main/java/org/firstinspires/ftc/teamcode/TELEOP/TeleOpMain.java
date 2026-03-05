package org.firstinspires.ftc.teamcode.TELEOP;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Arrays;

@Configurable
@TeleOp(name="TeleOpMain")

/*
 Controls:
   gamepad1:
     - left stick up/down  = forward/backward
     - right stick left/right = rotate
     - left/right triggers = side
     - x, a, b = push left, mid, right ball
     - y = push all 3 balls

   gamepad2:
     - left stick up/down = intake in/out
     - right stick up = shoot
     - x, a, b = select pattern
     - y = shoot by pattern (press again to cancel)
     - dpad = shooter selection
     - bumpers = adjust shooting speed (+- 0.1)
*/

public class TeleOpMain extends LinearOpMode {

    // -------------------- CONSTANTS --------------------
    static final double servoOpen = 1;
    static final double servoPush = 0;
    double SPEEDFACTOR = 0.55;
    static final double SPEEDROTATE = 0.35;
    static double shootingSpeed = 0.85;

    String[] shootersToPower = {"L", "M", "R"};
    String[] PATTERN = {"Purple,", "Purple", "Green"};

    DcMotor shooterL, shooterM, shooterR;

    // Bumper tracking
    boolean rightBumperPrev = false;
    boolean leftBumperPrev = false;

    // Drive reverse toggle
    boolean driveReversed = false;
    boolean leftBumperPrevDrive = false;

    // Pattern toggle system
    boolean shootingByPattern = false;
    boolean yPrev = false;
    int patternIndex = 0;
    boolean shooterSpunUp = false;
    long spinUpStartTime = 0;

    public void runOpMode() {

        // -------------------- HARDWARE INIT --------------------
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "left_front");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "left_back");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "right_front");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "right_back");

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotor.class, "shooter_left");
        shooterM = hardwareMap.get(DcMotor.class, "shooter_mid");
        shooterR = hardwareMap.get(DcMotor.class, "shooter_right");

        Servo servoL = hardwareMap.get(Servo.class, "servo_left");
        Servo servoM = hardwareMap.get(Servo.class, "servo_mid");
        Servo servoR = hardwareMap.get(Servo.class, "servo_right");

        NormalizedColorSensor colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "ball_color_left");
        NormalizedColorSensor colorSensorM = hardwareMap.get(NormalizedColorSensor.class, "ball_color_mid");
        NormalizedColorSensor colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "ball_color_right");

        // -------------------- MOTOR DIRECTIONS --------------------
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        servoR.setDirection(Servo.Direction.REVERSE);
        shooterM.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoOpen);

        String left_ball = "No Color";
        String mid_ball = "No Color";
        String right_ball = "No Color";

        double forward, rotation, side = 0, intakeSpeed, shooterStick;

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {

            // ----------------- DRIVER INPUT -----------------
            forward = gamepad1.left_stick_y;
            rotation = gamepad1.right_stick_x;
            intakeSpeed = -gamepad2.left_stick_y;
            shooterStick = -gamepad2.right_stick_y;


            // -------------------- DRIVE REVERSE TOGGLE --------------------
            if (gamepad1.left_bumper && !leftBumperPrevDrive) {
                driveReversed = !driveReversed;
            }
            leftBumperPrevDrive = gamepad1.left_bumper;

            if (driveReversed) {
                SPEEDFACTOR = -SPEEDFACTOR;
            }

            // -------------------- CALCULATE SIDE --------------------
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                side = gamepad1.right_trigger - gamepad1.left_trigger;
            } else if (gamepad1.left_trigger > 0) {
                side = -gamepad1.left_trigger;
            } else if (gamepad1.right_trigger > 0) {
                side = gamepad1.right_trigger;
            }

            // -------------------- MANUAL SERVO PUSH --------------------
            if (gamepad1.x) { servoL.setPosition(servoPush); sleep(1000); servoL.setPosition(servoOpen); }
            if (gamepad1.a) { servoM.setPosition(servoPush); sleep(1000); servoM.setPosition(servoOpen); }
            if (gamepad1.b) { servoR.setPosition(servoPush); sleep(1000); servoR.setPosition(servoOpen); }
            if (gamepad1.y) {
                servoL.setPosition(servoPush); servoM.setPosition(servoPush); servoR.setPosition(servoPush);
                sleep(1000);
                servoL.setPosition(servoOpen); servoM.setPosition(servoOpen); servoR.setPosition(servoOpen);
            }

            // -------------------- SHOOTER MODE SELECTION --------------------
            if (gamepad2.dpad_left) shootersToPower = new String[]{"L"};
            else if (gamepad2.dpad_down) shootersToPower = new String[]{"M"};
            else if (gamepad2.dpad_right) shootersToPower = new String[]{"R"};
            else if (gamepad2.dpad_up) shootersToPower = new String[]{"L", "M", "R"};

            // -------------------- PATTERN SELECTION --------------------
            if (gamepad2.xWasPressed()) PATTERN = new String[]{"Green", "Purple", "Purple"};
            else if (gamepad2.aWasPressed()) PATTERN = new String[]{"Purple", "Green", "Purple"};
            else if (gamepad2.bWasPressed()) PATTERN = new String[]{"Purple", "Purple", "Green"};

            // -------------------- ADJUST SHOOTING SPEED --------------------
            if (gamepad2.right_bumper && !rightBumperPrev) {
                shootingSpeed += 0.05;
                if (shootingSpeed > 1.0) shootingSpeed = 1.0;
            }
            rightBumperPrev = gamepad2.right_bumper;

            if (gamepad2.left_bumper && !leftBumperPrev) {
                shootingSpeed -= 0.05;
                if (shootingSpeed < 0.0) shootingSpeed = 0.0;
            }
            leftBumperPrev = gamepad2.left_bumper;

            // -------------------- TOGGLE SHOOT BY PATTERN --------------------
            if (gamepad2.y && !yPrev) {
                shootingByPattern = !shootingByPattern;

                if (!shootingByPattern) {
                    shooterL.setPower(0);
                    shooterM.setPower(0);
                    shooterR.setPower(0);
                } else {
                    patternIndex = 0;
                    shooterSpunUp = false;
                    spinUpStartTime = 0;
                }
            }
            yPrev = gamepad2.y;

            // -------------------- SHOOT BY PATTERN LOGIC --------------------
            if (shootingByPattern) {

                String[] balls = {left_ball, mid_ball, right_ball};
                Servo[] servos = {servoL, servoM, servoR};

                if (!shooterSpunUp) {

                    shooterL.setPower(shootingSpeed);
                    shooterM.setPower(-shootingSpeed);
                    shooterR.setPower(-shootingSpeed);

                    if (spinUpStartTime == 0)
                        spinUpStartTime = System.currentTimeMillis();

                    if (System.currentTimeMillis() - spinUpStartTime > 3000)
                        shooterSpunUp = true;

                } else if (patternIndex < PATTERN.length) {

                    for (int j = 0; j < balls.length; j++) {
                        if (balls[j].equals(PATTERN[patternIndex])) {
                            servos[j].setPosition(servoPush);
                            sleep(800);
                            servos[j].setPosition(servoOpen);
                            patternIndex++;
                            break;
                        }
                    }

                } else {
                    shootingByPattern = false;
                    shooterL.setPower(0);
                    shooterM.setPower(0);
                    shooterR.setPower(0);
                }
            }

            // -------------------- SHOOTER POWER LOGIC --------------------
            double powerL = 0, powerM = 0, powerR = 0;
            for (String s : shootersToPower) {
                if (s.equals("L")) powerL = shooterStick;
                if (s.equals("M")) powerM = -shooterStick;
                if (s.equals("R")) powerR = -shooterStick;
            }

            leftFront.setPower(forward*SPEEDFACTOR - rotation*SPEEDROTATE - side*SPEEDFACTOR);
            leftBack.setPower(forward*SPEEDFACTOR - rotation*SPEEDROTATE + side*SPEEDFACTOR);
            rightFront.setPower(forward*SPEEDFACTOR + rotation*SPEEDROTATE + side*SPEEDFACTOR);
            rightBack.setPower(forward*SPEEDFACTOR + rotation*SPEEDROTATE - side*SPEEDFACTOR);

            intake.setPower(intakeSpeed);

            if (!shootingByPattern) {
                shooterL.setPower(powerL * shootingSpeed);
                shooterM.setPower(powerM * shootingSpeed);
                shooterR.setPower(powerR * shootingSpeed);
            }

            NormalizedRGBA colors_left = colorSensorL.getNormalizedColors();
            NormalizedRGBA colors_mid = colorSensorM.getNormalizedColors();
            NormalizedRGBA colors_right = colorSensorR.getNormalizedColors();

            left_ball  = (colors_left.blue > colors_left.green)  ? "Purple" : "Green";
            mid_ball   = (colors_mid.blue > colors_mid.green)   ? "Purple" : "Green";
            right_ball = (colors_right.blue > colors_right.green) ? "Purple" : "Green";

            telemetry.addData("Pattern: ", Arrays.toString(PATTERN));
            telemetry.addData("Shooter mode: ", Arrays.toString(shootersToPower));
            telemetry.addData("Left: ", left_ball);
            telemetry.addData("Mid: ", mid_ball);
            telemetry.addData("Right: ", right_ball);
            telemetry.addData("Shooting Speed: ", shootingSpeed);
            telemetry.addData("Pattern Active: ", shootingByPattern);
            telemetry.addData("Drive Reversed:", driveReversed);
            telemetry.update();
        }
    }
}