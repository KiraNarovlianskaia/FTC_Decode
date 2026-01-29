package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
@TeleOp(name="Sasha TeleOP Sasha")

public class TeleOpSasha extends LinearOpMode {


    DcMotor leftFront;

    DcMotor leftBack;

    DcMotor rightFront;
    

    DcMotor rightBack;

    DcMotor intake;




    static final double SPEEDFACTOR = 0.3;

    static final double SPEEDINTAKE = 0.5;


    public void runOpMode(){


        double forward;

        double rotation;

        double side;

        double intaker;

        double intakel;



        leftFront = hardwareMap.get(DcMotor.class, "left_front");

        leftBack = hardwareMap.get(DcMotor.class, "left_back");

        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        intake = hardwareMap.get(DcMotor.class, "shooter");


        leftFront.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setDirection(DcMotor.Direction.FORWARD);

        rightFront.setDirection(DcMotor.Direction.REVERSE);

        rightBack.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();


        while (opModeIsActive()) {

            forward = gamepad1.left_stick_y;


            side = gamepad1.left_stick_x;


            rotation = gamepad1.right_stick_x;


            intaker = gamepad2.right_trigger;


            intakel = gamepad2.left_trigger;


            leftFront.setPower((forward - rotation - side) * SPEEDFACTOR);

            leftBack.setPower((forward - rotation + side)* SPEEDFACTOR);

            rightFront.setPower((forward + rotation + side)* SPEEDFACTOR);

            rightBack.setPower((forward + rotation - side)* SPEEDFACTOR);

            intake.setPower((intaker - intakel)* SPEEDINTAKE);


            sleep(50);

        }



    }




}

