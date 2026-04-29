package org.firstinspires.ftc.teamcode.AUTO.Anna;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@Autonomous(name="Red Up", group="Red")
public class RedUp extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private IMU imu;

    private DcMotor shooterLeft;
    private DcMotor shooterMid;
    private DcMotor shooterRight;
    private Servo servoLeft;
    private Servo servoMid;
    private Servo servoRight;

    static final double PI = 3.1415;

    static final double DIAMETER = 9.6;

    static final double PULSE = 537.7;
    static final double PULSE_PER_CM = PULSE / (PI * DIAMETER);

    public void runOpMode() {

        RevHubOrientationOnRobot.LogoFacingDirection LogoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

        RevHubOrientationOnRobot.UsbFacingDirection UsbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(LogoDirection, UsbDirection);


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooterLeft = hardwareMap.get(DcMotor.class, "Shooter_left");
        shooterMid = hardwareMap.get(DcMotor.class, "Shooter_mid");
        shooterRight = hardwareMap.get(DcMotor.class, "Shooter_right");

        servoLeft = hardwareMap.get(Servo.class,"Servo_left");
        servoMid = hardwareMap.get(Servo.class,"Servo_mid");
        servoRight = hardwareMap.get(Servo.class,"Servo_right");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        waitForStart();

        servoRight.setPosition(0);
        servoMid.setPosition(1.0);
        servoLeft.setPosition(1.0);

        shooterRight.setPower(-0.1);
        shooterMid.setPower(0.1);
        shooterLeft.setPower(0.1);
        intake.setPower(0.1);
        sleep(5000);

        driveForward(0.5, 60);
        rotate(0.2,90);
        side(0.2, 1500);
    }

    public void side (double sideSpeed, double distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(-sideSpeed);
        leftBack.setPower(sideSpeed);
        rightFront.setPower(sideSpeed);
        rightBack.setPower(-sideSpeed);

        while (opModeIsActive() && leftFront.getCurrentPosition() > -distance) {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }

    public void rotate  (double rotateSpeed, double angle)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && Math.abs(getHeading()) < angle)
        {
            leftFront.setPower(rotateSpeed);
            leftBack.setPower(rotateSpeed);
            rightFront.setPower(-rotateSpeed);
            rightBack.setPower(-rotateSpeed);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep( 1000);
    }
    public void driveForward (double forwardSpeed, double distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSE_PER_CM * distance)
        {
            leftFront.setPower(forwardSpeed);
            leftBack.setPower(forwardSpeed);
            rightFront.setPower(forwardSpeed);
            rightBack.setPower(forwardSpeed);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep( 1000);
    }
    public double getHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}