package org.firstinspires.ftc.teamcode.AUTO.Oleg;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous (name="Auto Blue From Wall")
public class AutoBlueFromWall extends LinearOpMode {


    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;
    DcMotor shooter;
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

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();

        shoot();

        moveRotate(-0.5, 72);
        moveSide(0.5, 60);
        intakeStart();
        moveForward(0.5, 70);
        intakeStop();
        moveForward(-0.5, 70);
        moveSide(-0.5, 60);
        moveRotate(0.5, 72);
        shoot();

        moveRotate(-0.5, 72);
        moveSide(0.5, 120);
        intakeStart();
        moveForward(0.5, 70);
        intakeStop();
        moveForward(-0.5, 70);
        moveSide(-0.5, 120);
        moveRotate(0.5, 72);
        shoot();

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

    public void shoot() {
        shooter.setPower(1);
        sleep(5000);
        shooter.setPower(0);
    }

    public void intakeStart() {
        intake.setPower(0.8);
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