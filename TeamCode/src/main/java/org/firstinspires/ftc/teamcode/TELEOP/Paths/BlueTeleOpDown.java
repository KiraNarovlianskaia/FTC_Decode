package org.firstinspires.ftc.teamcode.TELEOP.Paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.function.Supplier;

@Configurable
@TeleOp(name="BLUE TeleOp Down")

public class BlueTeleOpDown extends LinearOpMode {

    // ---------------- PEDRO PATHING ----------------
    private Follower follower;
    private boolean automatedDrive = false;

    private Supplier<PathChain> pathChainBase;
    private Supplier<PathChain> pathChainShoot;
    private Supplier<PathChain> pathChainShootDown;
    private Supplier<PathChain> pathChainHuman;

    // ---------------- CONSTANTS ----------------
    static final double servoOpen = 1;
    static final double servoPush = 0;

    static double shootingSpeed = 0.85;
    static double shootingSpeedM = shootingSpeed + 0.1;
    static double speed_factor = 0.8;

    String[] shootersToPower = {"L", "M", "R"};
    String[] PATTERN = {"Purple", "Purple", "Green"};

    // ---------------- HARDWARE ----------------
    DcMotor intake;
    DcMotor shooterL, shooterM, shooterR;

    Servo servoL, servoM, servoR;

    NormalizedColorSensor colorSensorL;
    NormalizedColorSensor colorSensorM;
    NormalizedColorSensor colorSensorR;

    // ---------------- STATE ----------------
    boolean shootingByPattern = false;
    boolean yPrev = false;
    boolean rbBefore = false;
    boolean lbBefore = false;

    int patternIndex = 0;

    boolean shooterSpunUp = false;
    long spinUpStart = 0;

    long servoTimer = 0;
    boolean servoActive = false;

    boolean rightBumperPrev = false;
    boolean leftBumperPrev = false;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE ----------------

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotor.class, "shooter_left");
        shooterM = hardwareMap.get(DcMotor.class, "shooter_mid");
        shooterR = hardwareMap.get(DcMotor.class, "shooter_right");

        servoL = hardwareMap.get(Servo.class, "servo_left");
        servoM = hardwareMap.get(Servo.class, "servo_mid");
        servoR = hardwareMap.get(Servo.class, "servo_right");

        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "ball_color_left");
        colorSensorM = hardwareMap.get(NormalizedColorSensor.class, "ball_color_mid");
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "ball_color_right");

        shooterM.setDirection(DcMotor.Direction.REVERSE);
        servoR.setDirection(Servo.Direction.REVERSE);

        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoOpen);

        // ---------------- PEDRO INIT ----------------

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(59.449, 33.213, Math.toRadians(-90)));

        pathChainBase = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(103.403, 32.773))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(90),
                                0.8))
                .build();

        pathChainShoot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(48.594, 94.574))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(315),
                                0.8))
                .build();
        pathChainShootDown = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72.104, 21.931))))//new shoot down
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(305),
                                0.8))
                .build();
        pathChainHuman = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(110.994, 11.824))))//new go to human player
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(0),
                                0.8))
                .build();

        waitForStart();

        follower.startTeleopDrive();

        String left_ball = "No Color";
        String mid_ball = "No Color";
        String right_ball = "No Color";

        // ================= MAIN LOOP =================

        while (opModeIsActive()) {

            follower.update();

            // ---------------- DRIVE ----------------

            if (!automatedDrive) {

                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * speed_factor,
                        (gamepad1.left_trigger - gamepad1.right_trigger) * speed_factor,
                        -gamepad1.right_stick_x * speed_factor,
                        true
                );

            }

            if (gamepad1.right_bumper && !rbBefore) {
                speed_factor = (speed_factor == 0.4) ? 0.8 : 0.4;
            }
            rbBefore = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !lbBefore) {
                speed_factor = -speed_factor;
            }
            lbBefore = gamepad1.left_bumper;


            if (gamepad1.dpad_down) {
                follower.followPath(pathChainBase.get());
                automatedDrive = true;
            }

            if (gamepad1.dpad_right) {
                follower.followPath(pathChainShootDown.get());
                automatedDrive = true;
            }
            if (gamepad1.dpad_left) {
                follower.followPath(pathChainHuman.get());
                automatedDrive = true;
            }
            if (gamepad1.dpad_up) {

                follower.followPath(pathChainShoot.get());
                automatedDrive = true;

            }

            if (gamepad2.x) {
                servoL.setPosition(servoPush);
                sleep(1000);
                servoL.setPosition(servoOpen);
            }
            if (gamepad2.a) {
                servoM.setPosition(servoPush);
                sleep(1000);
                servoM.setPosition(servoOpen);
            }
            if (gamepad2.b) {
                servoR.setPosition(servoPush);
                sleep(1000);
                servoR.setPosition(servoOpen);
            }
            if (gamepad2.y) {
                servoL.setPosition(servoPush);
                servoM.setPosition(servoPush);
                servoR.setPosition(servoPush);
                sleep(1000);
                servoL.setPosition(servoOpen);
                servoM.setPosition(servoOpen);
                servoR.setPosition(servoOpen);
            }

            if (automatedDrive && (gamepad1.a || !follower.isBusy())) {

                follower.startTeleopDrive();
                automatedDrive = false;

            }

            // ---------------- INTAKE ----------------

            intake.setPower(-gamepad2.left_stick_y);

            // ---------------- SHOOTER MANUAL ----------------

            double shooterStick = -gamepad2.right_stick_y;

            double powerL = 0;
            double powerM = 0;
            double powerR = 0;

            for (String s : shootersToPower) {

                if (s.equals("L")) powerL = shooterStick;
                if (s.equals("M")) powerM = -shooterStick;
                if (s.equals("R")) powerR = -shooterStick;

            }

            if (!shootingByPattern) {

                shooterL.setPower(powerL * shootingSpeed);
                shooterM.setPower(powerM * shootingSpeed);
                shooterR.setPower(powerR * shootingSpeed);

            }

            // ---------------- SPEED ADJUST ----------------

            if (gamepad2.right_bumper && !rightBumperPrev) {

                shootingSpeed += 0.05;
                if (shootingSpeed > 1) shootingSpeed = 1;

            }

            rightBumperPrev = gamepad2.right_bumper;

            if (gamepad2.left_bumper && !leftBumperPrev) {

                shootingSpeed -= 0.05;
                if (shootingSpeed < 0) shootingSpeed = 0;

            }

            leftBumperPrev = gamepad2.left_bumper;

            // ---------------- PATTERN SELECT ----------------

            if (gamepad2.xWasPressed())
                PATTERN = new String[]{"Green", "Purple", "Purple"};

            if (gamepad2.aWasPressed())
                PATTERN = new String[]{"Purple", "Green", "Purple"};

            if (gamepad2.bWasPressed())
                PATTERN = new String[]{"Purple", "Purple", "Green"};

            // ---------------- SHOOT MODE ----------------

            if (gamepad2.y && !yPrev) {

                shootingByPattern = !shootingByPattern;

                patternIndex = 0;
                shooterSpunUp = false;
                spinUpStart = 0;
                servoActive = false;

                if (!shootingByPattern) {

                    shooterL.setPower(0);
                    shooterM.setPower(0);
                    shooterR.setPower(0);

                }

            }

            yPrev = gamepad2.y;

            // ---------------- COLOR DETECTION ----------------

            NormalizedRGBA colors_left = colorSensorL.getNormalizedColors();
            NormalizedRGBA colors_mid = colorSensorM.getNormalizedColors();
            NormalizedRGBA colors_right = colorSensorR.getNormalizedColors();

            left_ball = (colors_left.blue > colors_left.green) ? "Purple" : "Green";
            mid_ball = (colors_mid.blue > colors_mid.green) ? "Purple" : "Green";
            right_ball = (colors_right.blue > colors_right.green) ? "Purple" : "Green";

            // ---------------- PATTERN SHOOT ----------------

            if (shootingByPattern) {

                String[] balls = {left_ball, mid_ball, right_ball};
                Servo[] servos = {servoL, servoM, servoR};

                // ---------- SPIN UP ----------

                if (!shooterSpunUp) {

                    shooterL.setPower(shootingSpeed);
                    shooterM.setPower(-shootingSpeed);
                    shooterR.setPower(-shootingSpeed);

                    if (spinUpStart == 0)
                        spinUpStart = System.currentTimeMillis();

                    if (System.currentTimeMillis() - spinUpStart > 2500)
                        shooterSpunUp = true;

                }

                // ---------- SHOOT PATTERN ----------

                else if (patternIndex < PATTERN.length) {

                    int targetIndex = -1;

                    for (int i = 0; i < balls.length; i++) {

                        if (PATTERN[patternIndex].equalsIgnoreCase(balls[i])) {

                            targetIndex = i;
                            break;

                        }

                    }

                    if (targetIndex == -1) {

                        patternIndex++;

                    } else {

                        if (!servoActive) {

                            servos[targetIndex].setPosition(servoPush);

                            servoTimer = System.currentTimeMillis();
                            servoActive = true;

                        } else if (System.currentTimeMillis() - servoTimer > 800) {

                            servos[targetIndex].setPosition(servoOpen);

                            servoActive = false;
                            patternIndex++;

                        }

                    }

                }

                // ---------- FINISH ----------

                else {

                    shootingByPattern = false;

                    shooterL.setPower(0);
                    shooterM.setPower(0);
                    shooterR.setPower(0);

                }

            }


            // ---------------- TELEMETRY ----------------

            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Pattern", Arrays.toString(PATTERN));
            telemetry.addData("Left", left_ball);
            telemetry.addData("Mid", mid_ball);
            telemetry.addData("Right", right_ball);
            telemetry.addData("ShootSpeed", shootingSpeed);
            telemetry.addData("AutoDrive", automatedDrive);

            telemetry.update();

        }

    }
}
