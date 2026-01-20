package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous (name="Anna Auto")
public class Anna extends LinearOpMode {


    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    static final double forward = 0.3;
    static final double PI = 3.14159265;
    static final double WHEEL_DIAMETER = 10.4;
    static final double PULSES = 537.7;
    static final double PULSES_PER_CM = PULSES / (PI * WHEEL_DIAMETER);
    static final double DISTANCE = 100;

    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        moveForward(-0.3, 160);
        // shoot
        moveRotate(0.3, -45);
        moveForward(0.3, 115); //collect
        moveForward(-0.3, 115);
        moveRotate(0.3, 45);
        //shoot
        moveRotate(0.3, -45);
        moveSide(0.3, 70);
        moveForward(0.3, 115); //collect
        moveForward(-0.3, 115);
        moveSide(-0.3, 70);
        moveRotate(0.3, 45);
        //shoot

    }
    double void moveForward( double speed, double distance){

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && leftFront.getCurrentPosition() < PULSES_PER_CM * distance) ;

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }
    double void moveRotate( double speed, double angle){

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(-speed);
        rightBack.setPower(-speed);

        while (opModeIsActive() && leftFront.getCurrentPosition() < angle);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }
    double void moveSide( double speed, double distance){

        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && leftFront.getCurrentPosition() < PULSES_PER_CM * distance) ;

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }
}
