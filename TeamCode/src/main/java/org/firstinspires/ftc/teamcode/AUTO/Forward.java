package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import static org.firstinspires.ftc.teamcode.AUTO.Globals.*;

@Autonomous(name="Auto 2 Specimen", group="Robot")
public class Forward extends LinearOpMode {
    @Override
    public void runOpMode() {
        touchVertical = hardwareMap.get(TouchSensor.class, "sensor_touch");

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        Vertical = hardwareMap.get(DcMotor.class, "Vertical");

        VerRotate = hardwareMap.get(Servo.class, "Vertical Rotate");
        VerRotate.setPosition(0.85);
        VerClaw = hardwareMap.get(Servo.class, "Vertical Claw");
        VerClaw.setPosition(0.5);

        HorRotate = hardwareMap.get(Servo.class, "Horizontal Rotate");
        HorRotate.setPosition(0.45);
        HorClaw = hardwareMap.get(Servo.class, "Horizontal Claw");
        HorClaw.setPosition(0.2);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        Vertical.setDirection(DcMotor.Direction.FORWARD);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        while (opModeInInit()) {

            telemetry.addData("Currently at:", "%4.0f", getHeading());
            telemetry.update();

        }

        waitForStart();

        telemetry.addData("Vertical", Vertical.getCurrentPosition());
        telemetry.update();


        driveStraight(0.2, 40);//move to the chamber


    }
    public void driveRotate(double rotateSpeed, double angle)
    {
        imu.resetYaw();

        while (opModeIsActive() && Math.abs(getHeading()) < angle) {

            telemetry.addData("Currently at:", "%4.0f", getHeading());
            telemetry.update();

            LeftFront.setPower(-rotateSpeed);
            LeftBack.setPower(-rotateSpeed);
            RightFront.setPower(rotateSpeed);
            RightBack.setPower(rotateSpeed);
        }
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
        sleep(300);
    }
    public void driveStraight(double driveSpeed, double distance)
    {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < PULSES_PER_CM * distance) {

            telemetry.addData("Currently at:", "%5d", LeftFront.getCurrentPosition());
            telemetry.update();

            LeftFront.setPower(driveSpeed);
            LeftBack.setPower(driveSpeed);
            RightFront.setPower(driveSpeed);
            RightBack.setPower(driveSpeed);

        }
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
        sleep(500);
    }
    public void driveSide(double driveSpeed, double distance)
    {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && Math.abs(LeftFront.getCurrentPosition()) < PULSES_PER_CM * distance) {

            telemetry.addData("Currently at:", "%5d", LeftFront.getCurrentPosition());
            telemetry.update();

            LeftFront.setPower(driveSpeed);
            LeftBack.setPower(-driveSpeed);
            RightFront.setPower(-driveSpeed);
            RightBack.setPower(driveSpeed);

        }
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
        sleep(500);
    }
    public void driveDiagonal(double driveSpeed, double distance)
    {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && Math.abs(RightFront.getCurrentPosition()) < PULSES_PER_CM * distance) {

            telemetry.addData("Currently at:", "%5d", LeftFront.getCurrentPosition());
            telemetry.update();

            LeftFront.setPower(0);
            LeftBack.setPower(-driveSpeed);
            RightFront.setPower(-driveSpeed);
            RightBack.setPower(0);

        }
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
        sleep(500);
    }
    public void VerSliderPosition(double position, double power){
        Vertical.setTargetPosition((int) position);
        Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Vertical.setPower(power);

        while (opModeIsActive() && Vertical.isBusy()) {
            telemetry.addData("Current Position", Vertical.getCurrentPosition());
            telemetry.update();
        }
    }

    public void VerSliderZero(double power){

        while (opModeIsActive() && !touchVertical.isPressed()) {
            Vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Vertical.setPower(power);  // Keep moving down
            telemetry.addData("Vertical Motor", "Moving Down");
            telemetry.update();
        }

        // Stop the vertical motor once the sensor is pressed
        Vertical.setPower(0);
    }
    public double getHeading()
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}