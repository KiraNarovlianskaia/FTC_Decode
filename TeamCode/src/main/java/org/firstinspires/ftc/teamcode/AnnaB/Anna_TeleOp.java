package org.firstinspires.ftc.teamcode.AnnaB;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="AnnaB TeleOp AnnaB")
    public class Anna_TeleOp extends LinearOpMode {


        DcMotor leftFront;
        DcMotor leftBack;
        DcMotor rightFront;
        DcMotor rightBack;




        static final double SPEEDFACTOR = 0.3;


        public void runOpMode(){


            double forward;
            double rotation;
            double side;


            leftFront = hardwareMap.get(DcMotor.class, "left_front");
            leftBack = hardwareMap.get(DcMotor.class, "left_back");
            rightFront = hardwareMap.get(DcMotor.class, "right_front");
            rightBack = hardwareMap.get(DcMotor.class, "right_back");



            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);



            waitForStart();


            while (opModeIsActive()) {

                forward = gamepad1.left_stick_y;
                side = gamepad1.left_stick_x;
                rotation = gamepad1.right_stick_x;



                leftFront.setPower((forward - rotation - side) * SPEEDFACTOR);
                leftBack.setPower((forward - rotation + side)* SPEEDFACTOR);
                rightFront.setPower((forward + rotation + side)* SPEEDFACTOR);
                rightBack.setPower((forward + rotation - side)* SPEEDFACTOR);



            }
        }
    }


