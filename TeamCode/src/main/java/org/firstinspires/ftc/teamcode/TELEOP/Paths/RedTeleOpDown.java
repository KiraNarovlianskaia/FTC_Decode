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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name="RED Down TeleOp")

public class RedTeleOpDown extends LinearOpMode {

    // ---------------- PEDRO PATHING ----------------
    private Follower follower;
    private boolean automatedDrive = false;

    private Supplier<PathChain> pathChainBase;
    private Supplier<PathChain> pathChainShoot;
    private Supplier<PathChain> pathChainShootDown;
    private Supplier<PathChain> pathChainHuman;

    // ---------------- CONSTANTS ----------------
    static final double servoOpen = 0;
    static final double servoPush = 0.5;

    static double shootingdown = 0.8;
    static double shootingup = 0.6;
    static double shootingSpeed = 0.85;

    static double speed_factor = 0.8;

    // ---------------- HARDWARE ----------------
    DcMotor intake;
    DcMotor shooterL, shooterM, shooterR;

    Servo servoL, servoM, servoR;

    NormalizedColorSensor colorSensorL;
    NormalizedColorSensor colorSensorM;
    NormalizedColorSensor colorSensorR;

    // ---------------- STATE ----------------
    boolean rbBefore = false;
    boolean lbBefore = false;
    boolean shootingByPattern = false;
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

        servoR.setDirection(Servo.Direction.REVERSE);
        //shooterM.setDirection(DcMotor.Direction.REVERSE);
        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoOpen+0.5);

        // ---------------- PEDRO INIT ----------------

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(84.551, 33.213, Math.toRadians(270))); //not decided yet

        pathChainBase = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.559, 33.490))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(90),
                                0.8))
                .build();

        pathChainShoot = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(96.024, 95.652))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(225),
                                0.8))
                .build();

        pathChainShootDown = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(82.385, 20.968))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(245),
                                0.8))
                .build();

        pathChainHuman = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(29.291, 10.509))))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(180),
                                0.8))
                .build();

        waitForStart();

        follower.startTeleopDrive();


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

            if (gamepad1.dpad_right) {
                follower.followPath(pathChainBase.get());
                automatedDrive = true;
                shootingSpeed = shootingup;
            }

            if (gamepad1.dpad_down) {
                follower.followPath(pathChainShootDown.get());
                automatedDrive = true;
                shootingSpeed = shootingdown;
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
                servoR.setPosition(servoOpen);
                sleep(1000);
                servoR.setPosition(servoPush);
            }

            if (gamepad2.y) {
                servoL.setPosition(servoPush);
                servoM.setPosition(servoPush);
                servoR.setPosition(servoOpen);
                sleep(1000);
                servoL.setPosition(servoOpen);
                servoM.setPosition(servoOpen);
                servoR.setPosition(servoPush);
            }

            if (automatedDrive && (gamepad1.a || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }

            // ---------------- INTAKE ----------------

            intake.setPower(-gamepad2.left_stick_y);

            // ---------------- SHOOTER MANUAL ----------------

            double shooterStick = gamepad2.right_stick_y;
            shooterL.setPower(shooterStick * shootingSpeed);
            shooterM.setPower(shooterStick * shootingSpeed);
            shooterR.setPower(shooterStick * shootingSpeed);

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

            // ---------------- TELEMETRY ----------------

            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("ShootSpeed", shootingSpeed);
            telemetry.addData("AutoDrive", automatedDrive);

            telemetry.update();

        }

    }
}
