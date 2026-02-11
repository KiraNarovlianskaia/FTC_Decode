package org.firstinspires.ftc.teamcode.AUTO.Sasha;

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



@Disabled
@Autonomous (name="Auto Sasha Red Wall No Teammate", group = "Sasha")
public class AutoSashaRedWallNoTeammate extends LinearOpMode {


    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;

    DcMotor shoot;
    Servo servoL;
    Servo servoR;

    private IMU imu = null;
    static final double forward = 0.3;
    static final double PI = 3.14159265;
    static final double WHEEL_DIAMETER = 10.4;
    static final double PULSES = 537.7;
    static final double PULSES_PER_CM = PULSES / (PI * WHEEL_DIAMETER);

    static final double SERVO_L_OPEN = 0.45;
    static final double SERVO_L_CLOSED = 0;

    static final double SERVO_R_OPEN = 0.45;
    static final double SERVO_R_CLOSED = 0;


    public void runOpMode(){

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shoot = hardwareMap.get(DcMotor.class, "shooter");

        servoL = hardwareMap.get(Servo.class, "left_servo");
        servoR = hardwareMap.get(Servo.class, "right_servo");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shoot.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.resetYaw();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeInInit()){
            telemetry.addData(">","Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();
        shooter();
        servoClose();
        sleep(3000);
        servoOpen();
        sleep(500);
        stopshoot();
        servoClose();
        rotate(-0.4,25);
        forward(-0.4,40);
        rotate(-0.4,90);
        intaker();
        forward(0.4,90);
        sleep(500);
        forward(-0.4,90);
        stopintake();
        shooter();
        rotate(0.4,90);
        forward(-0.4,40);
        rotate(0.4, 25);
        servoOpen();
        sleep(1000);
        stopshoot();
        servoClose();
        rotate(-0.4,25);
        forward(-0.4,120);
        rotate(-0.4,90);
        intaker();
        forward(0.4,90);
        sleep(500);
        forward(-0.4,90);
        stopintake();
        shooter();
        rotate(0.4,90);
        forward(0.4,45);
        rotate(0.4,40);
        servoOpen();
        sleep(1000);
        stopshoot();
        servoClose();
        forward(0.4,10);




    }
    public void forward( double speed, double distance){

        encoder();

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSES_PER_CM * distance);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }

    public void rotate( double speed1, double angle1){

        imu.resetYaw();

        leftFront.setPower(speed1);
        leftBack.setPower(speed1);
        rightFront.setPower(-speed1);
        rightBack.setPower(-speed1);

        while (opModeIsActive() &&
                Math.abs(getHeading()) < Math.abs(angle1)) {
            idle();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);

    }


    public void side( double speed2, double distance1){

        encoder();

        leftFront.setPower(speed2);
        leftBack.setPower(-speed2);
        rightFront.setPower(-speed2);
        rightBack.setPower(speed2);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSES_PER_CM * distance1);

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

    public void encoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intaker(){
        intake.setPower(0.8);
    }
    public void stopintake(){
        intake.setPower(0);
        sleep(500);
    }

    public void shooter(){
        shoot.setPower(0.7875);
    }
    public void stopshoot(){
        shoot.setPower(0);
        sleep(500);
    }

    public void servoOpen(){
        servoL.setPosition(SERVO_L_OPEN);
        servoR.setPosition(SERVO_R_OPEN);
    }

    public void servoClose(){
        servoL.setPosition(SERVO_L_CLOSED);
        servoR.setPosition(SERVO_R_CLOSED);
    }
}