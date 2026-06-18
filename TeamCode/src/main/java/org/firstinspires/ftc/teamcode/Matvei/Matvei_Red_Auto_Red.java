package org.firstinspires.ftc.teamcode.Matvei;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Matvei Auto Red")
public class Matvei_Red_Auto_Red extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intakeMotor;
    private DcMotor shooterL;
    private DcMotor shooterM;
    private DcMotor shooterR;
    private IMU imu;

    private Servo servoLeft;
    private Servo servoMid;
    private Servo servoRight;

    static final double sidedistance = -25;
    static  final double PI  = 3.1415;

    static  final double DIAMETER  = 9.6;

    static  final double PULSE =537.7;
    static  final double PULSE_PER_CM =PULSE/(PI * DIAMETER);

    public void runOpMode() {

        RevHubOrientationOnRobot.LogoFacingDirection LogoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

        RevHubOrientationOnRobot.UsbFacingDirection UsbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(LogoDirection, UsbDirection);


        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        shooterL = hardwareMap.get(DcMotor.class, "shooter_left");
        shooterM = hardwareMap.get(DcMotor.class, "shooter_mid");
        shooterR = hardwareMap.get(DcMotor.class, "shooter_right");

        servoLeft = hardwareMap.get(Servo.class,"servo_left");
        servoMid = hardwareMap.get(Servo.class,"servo_mid");
        servoRight = hardwareMap.get(Servo.class,"servo_right");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterM.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        servoRight.setPosition(0);
        servoMid.setPosition(0);
        servoLeft.setPosition(0);

        driveForward(0.5, 30);
        imu.resetYaw();
        shoot();
        rotate(0.2, 45);
        sideLeft(0.5, 15);
        intakeMotor.setPower(1);
        driveForward(0.5, 30);
        sleep(100);
        driveForward(-0.5, 30);
        intakeMotor.setPower(0);
        sideRight(0.5, 15);
        imu.resetYaw();
        rotate(-0.2, 45);
        shoot();
        imu.resetYaw();
        rotate(0.2, 44);
        sideLeft(0.5, 15);
        driveForward(0.5, 50);
        intakeMotor.setPower(1);
        driveForward(0.5, 20);
        sleep(100);
        intakeMotor.setPower(0);
        driveForward(-0.5, 70);
        sideRight(0.5, 15);
        imu.resetYaw();
        rotate(-0.2, 44);
        shoot();
        imu.resetYaw();
        rotate(0.2, 44);
        sideLeft(0.5, 15);
        driveForward(0.5, 70);
        intakeMotor.setPower(1);
        driveForward(0.5, 30);
        sleep(100);
        intakeMotor.setPower(0);
        driveForward(-0.5, 100);
        sideRight(0.5, 15);
        imu.resetYaw();
        rotate(-0.2, 44);
        shoot();

    }

    public void rotate  (double rotateSpeed, double angle)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && Math.abs(getHeading()) < angle)
        {
            leftFront.setPower(-rotateSpeed);
            leftBack.setPower(-rotateSpeed);
            rightFront.setPower(rotateSpeed);
            rightBack.setPower(rotateSpeed);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep( 100);
    }
    public void driveForward (double forwardSpeed, double distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        sleep( 100);
    }
    public void shoot ()
    {
        shooterR.setPower(-0.2);
        shooterM.setPower(0.2);
        shooterL.setPower(0.2);
        intakeMotor.setPower(1);
        sleep(1000);
        servoRight.setPosition(1.0);
        servoMid.setPosition(1.0);
        servoLeft.setPosition(1.0);
        sleep(1000);
        shooterR.setPower(0);
        shooterM.setPower(0);
        shooterL.setPower(0);
        intakeMotor.setPower(0);
        servoRight.setPosition(0);
        servoMid.setPosition(0);
        servoLeft.setPosition(0);
    }
    public void sideLeft (double sideSpeed, double distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && leftFront.getCurrentPosition() > sidedistance*distance)
        {
            leftFront.setPower(-sideSpeed);
            leftBack.setPower(sideSpeed);
            rightFront.setPower(sideSpeed);
            rightBack.setPower(-sideSpeed);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep( 100);
    }

    public void sideRight (double sideSpeed, double distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && leftFront.getCurrentPosition() < -sidedistance*distance)
        {
            leftFront.setPower(sideSpeed);
            leftBack.setPower(-sideSpeed);
            rightFront.setPower(-sideSpeed);
            rightBack.setPower(sideSpeed);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep( 100);
    }
    public double getHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}