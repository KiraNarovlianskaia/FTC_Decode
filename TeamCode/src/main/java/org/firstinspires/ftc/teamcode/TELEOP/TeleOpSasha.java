package org.firstinspires.ftc.teamcode.TELEOP;
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

    DcMotor shooter;




    static final double SPEEDFACTOR = 0.3;

    static final double SPEEDINTAKE = 0.8;

    static final double SPEEDSHOOT = 1.0;


    public void runOpMode(){


        double forward;

        double rotation;

        double side;

        double intaker;

        double intakel;

        double shoter;

        boolean shotup;

        boolean shotdown;



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



        waitForStart();


        while (opModeIsActive()) {

            forward = gamepad1.left_stick_y;


            side = gamepad1.left_stick_x;


            rotation = gamepad1.right_stick_x;


            intaker = gamepad2.right_trigger;


            intakel = gamepad2.left_trigger;

            shoter = gamepad2.left_stick_y;

            //shotdown = gamepad2.b;

            //shotup = gamepad2.a;




            leftFront.setPower((forward - rotation - side) * SPEEDFACTOR);

            leftBack.setPower((forward - rotation + side)* SPEEDFACTOR);

            rightFront.setPower((forward + rotation + side)* SPEEDFACTOR);

            rightBack.setPower((forward + rotation - side)* SPEEDFACTOR);

            intake.setPower((intaker - intakel));

            shooter.setPower((shoter)*SPEEDSHOOT);


            sleep(50);

        }



    }




}

