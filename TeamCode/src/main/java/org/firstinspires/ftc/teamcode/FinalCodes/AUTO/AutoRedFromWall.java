package org.firstinspires.ftc.teamcode.FinalCodes.AUTO;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous (name="Auto Red From Wall")
public class AutoRedFromWall extends LinearOpMode {


    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;
    DcMotor shooter;
    Servo servoL;
    Servo servoR;
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
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        servoL = hardwareMap.get(Servo.class, "left_servo");
        servoR = hardwareMap.get(Servo.class, "right_servo");



        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();

        shooterStart();
        sleep(1000);
        pushBalls();
        sleep(1500);
        shooterStop();

        moveRotate(0.35, 15);
        moveForward(0.35, 40);
        intakeStart();
        moveRotate(0.35, 90);
        moveForward(-0.25, 45);
        intakeStop();
        moveForward(0.35, 45);
        moveRotate(-0.35, 90);
        shooterStart();
        moveForward(-0.35, 40);
        moveRotate(-0.35, 15);
        // pushBall();
        sleep(2000);
        shooterStop();

        moveRotate(0.35, 15);
        moveForward(0.35, 115);
        moveRotate(0.35, 90);
        intakeStart();
        moveForward(-0.25, 50);
        intakeStop();
        moveForward(0.35, 50);
        moveRotate(-0.35, 90);
        moveForward(-0.35, 115);
        moveRotate(-0.35, 15);


    }
    public void moveForward(double speed, double distance) {

        resetEncoders();

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSES_PER_CM * distance) ;

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

    public void pushBalls() {
        servoL.setPosition(0.45);
        servoR.setPosition(0.45);
        sleep(400);
        servoL.setPosition(0.);
        servoR.setPosition(0);

    }


    public void shooterStart() {
        shooter.setPower(0.85);
    }

    public void shooterStop() {
        shooter.setPower(0);
    }

    public void intakeStart() {
        intake.setPower(0.9);
    }

    public void intakeStop() {
        intake.setPower(0.0);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}