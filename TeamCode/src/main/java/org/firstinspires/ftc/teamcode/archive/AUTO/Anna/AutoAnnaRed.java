package org.firstinspires.ftc.teamcode.archive.AUTO.Anna;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous (name="Anna Auto Red Goal 3 Artifacts", group = "Anna")
public class AutoAnnaRed extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;
    DcMotor shooterLeft;
    DcMotor shooterMid;
    DcMotor shooterRight;
    Servo servoLeft;
    Servo servoMid;
    Servo servoRight;

    static final double PI = 3.14159265;
    static final double WHEEL_DIAMETER = 10.4;
    static final double PULSES = 537.7;
    static final double PULSES_PER_CM = PULSES / (PI * WHEEL_DIAMETER);
    private IMU imu = null;


    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooter_left");
        shooterMid = hardwareMap.get(DcMotor.class, "shooter_mid");
        shooterRight = hardwareMap.get(DcMotor.class, "shooter_right");

        servoLeft = hardwareMap.get(Servo.class, "servo_left");
        servoMid = hardwareMap.get(Servo.class, "servo_mid");
        servoRight = hardwareMap.get(Servo.class, "servo_right");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        servoRight.setPosition(1.0);
        servoMid.setPosition(1.0);
        servoLeft.setPosition(1.0);

        waitForStart();

        moveForward(-0.3, 50);
        startShoot();
        servoClosed();
        sleep(1000);
        stopShoot();
        servoOpen();
        moveRotate(0.2, 35);
        moveSide(0.3, 40);
        startIntake();
        moveForward(-0.3, 15);
        moveForward(0.3, 15);
        stopIntake();
        moveSide(-0.3, 40);
        moveRotate(-0.2, 35); //it works! :)
        startShoot();
        servoClosed();
        sleep(1000);
        stopShoot();
        servoOpen();

        moveRotate(0.2, 35);
        moveForward(-0.3, 30);
        moveSide(0.3, 40);
        startIntake();
        moveForward(-0.3, 15);
        moveForward(0.3, 40);
        stopIntake();
        moveSide(-0.3, 40);
        moveRotate(-0.2, 35); //works too! :D
        startShoot();
        servoClosed();
        sleep(1000);
        stopShoot();
        servoOpen();
    }
    public void startShoot(){
        shooterLeft.setPower(1.0);
        shooterMid.setPower(1.0);
        shooterRight.setPower(1.0);
        sleep(500);
    }
    public void stopShoot(){
        shooterLeft.setPower(0);
        shooterMid.setPower(0);
        shooterRight.setPower(0);
    }
    public void startIntake(){
        intake.setPower(1.0);
    }
    public void stopIntake(){
        intake.setPower(0);
    }

    public void servoOpen(){
        servoLeft.setPosition(1.0);
        servoMid.setPosition(1.0);
        servoRight.setPosition(1.0);
    }
    public void servoClosed(){
        servoLeft.setPosition(0);
        servoMid.setPosition(0);
        servoRight.setPosition(0);
    }
    public void moveForward(double speed, double distance) {

        resetEncoders();

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSES_PER_CM * distance) {
            idle();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }
    public void moveRotate(double speed, double angle) {

        imu.resetYaw();

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(-speed);
        rightBack.setPower(-speed);

        while (opModeIsActive() &&
                Math.abs(getHeading()) < Math.abs(angle)) {
            idle();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }
    public void moveSide(double speed, double distance) {

        resetEncoders();

        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSES_PER_CM * distance) {
            idle();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetEncoders() {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}