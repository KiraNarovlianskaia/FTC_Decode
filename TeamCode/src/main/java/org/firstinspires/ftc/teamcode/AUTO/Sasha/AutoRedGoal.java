package org.firstinspires.ftc.teamcode.AUTO.Sasha;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous (name="Auto Red Goal")
public class AutoRedGoal extends LinearOpMode {


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
    static final double SERVO_L_OPEN = 0.45;
    static final double SERVO_L_CLOSED = 0;

    static final double SERVO_R_OPEN = 0.45;
    static final double SERVO_R_CLOSED = 0;


    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        servoL = hardwareMap.get(Servo.class, "left_servo");
        servoR = hardwareMap.get(Servo.class, "right_servo");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();

        shooter();
        servoClose();
        moveForward(0.3, 95);
        servoOpen();
        sleep(1000);
        stopshoot();
        servoClose();
        moveRotate(-0.3, 135);
        intakeStart();
        moveForward(0.3, 60);
        sleep(500);
        moveForward(-0.3, 60);
        intakeStop();
        shooter();
        moveRotate(0.3, 135);
        servoOpen();
        sleep(1000);
        stopshoot();
        servoClose();
        moveRotate(-0.3, 135);
        moveSide(0.3, 50);
        intakeStart();
        moveForward(0.3, 60); //collect
        moveForward(-0.3, 60);
        intakeStop();
        moveSide(-0.3, 50);
        shooter();
        moveRotate(0.3, 135);
        servoOpen();
        sleep(1000);
        stopshoot();
        servoClose();



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
    public void moveSide(double speed, double distance) {

        resetEncoders();

        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSES_PER_CM * distance) ;

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }


    public void intakeStart() {
        intake.setPower(0.6);
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
    public void servoOpen(){
        servoL.setPosition(SERVO_L_OPEN);
        servoR.setPosition(SERVO_R_OPEN);
    }

    public void servoClose(){
        servoL.setPosition(SERVO_L_CLOSED);
        servoR.setPosition(SERVO_R_CLOSED);
    }
    public void shooter(){
        shooter.setPower(-0.9);
    }
    public void stopshoot(){
        shooter.setPower(0);
        sleep(500);
    }
}