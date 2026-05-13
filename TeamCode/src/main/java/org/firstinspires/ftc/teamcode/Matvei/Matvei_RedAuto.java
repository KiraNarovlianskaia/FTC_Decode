package org.firstinspires.ftc.teamcode.Matvei;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Leave Red")
public class Matvei_RedAuto extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intakeMotor;
    private DcMotor shooterL;
    private DcMotor shooterM;
    private DcMotor shooterR;
    private DcMotor shooterMotor;
    private IMU imu;


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

        imu.resetYaw();
        driveForward(-0.5, 90);

    }

    public void rotate  (double rotateSpeed, double angle)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        sleep( 400);
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
        sleep( 100);
    }
    public double getHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}