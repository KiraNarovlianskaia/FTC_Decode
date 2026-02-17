package org.firstinspires.ftc.teamcode.FinalCodes.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@Configurable
@TeleOp(name="TeleOpFinal")

public class TeleOpSemiFinal extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;
    DcMotor shooter;
    Servo servoL;
    Servo servoR;



    static final double SERVO_L_OPEN = 0.45;
    static final double SERVO_L_CLOSED = 0;

    static final double SERVO_R_OPEN = 0.45;
    static final double SERVO_R_CLOSED = 0;



    static final double SPEEDFACTOR = 0.55;
    static final double SPEEDINTAKE = 0.9;
    static double SPEEDSHOOT = 0.85;


    public void runOpMode(){


        double forward;
        double rotation;
        double side;
        double intaker;
        double intakel;
        double shoter;


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        servoL = hardwareMap.get(Servo.class, "left_servo");
        servoR = hardwareMap.get(Servo.class, "right_servo");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        servoL.setPosition(SERVO_L_CLOSED);
        servoR.setPosition(SERVO_R_CLOSED);


        while (opModeIsActive()) {


            // Open and close left servo
            if (gamepad2.xWasPressed()) {
                servoL.setPosition(SERVO_L_OPEN);
                sleep(400);
                servoL.setPosition(SERVO_L_CLOSED);
            }

            // Open and close right servo
            if (gamepad2.bWasPressed()) {
                servoR.setPosition(SERVO_R_OPEN);
                sleep(400);
                servoR.setPosition(SERVO_R_CLOSED);
            }

            // Open and close both servos
            if (gamepad2.yWasPressed()) {
                servoL.setPosition(SERVO_L_OPEN);
                servoR.setPosition(SERVO_R_OPEN);
                sleep(400);
                servoL.setPosition(SERVO_L_CLOSED);
                servoR.setPosition(SERVO_R_CLOSED);
            }

            if (gamepad2.right_bumper) {
                if (SPEEDSHOOT == 0.85) {
                    SPEEDSHOOT = 0.5;
                } else {
                    SPEEDSHOOT = 0.85;
                }
            }

            forward = gamepad1.left_stick_y;
            side = gamepad1.left_stick_x;
            rotation = gamepad1.right_stick_x;
            intaker = gamepad2.right_trigger;
            intakel = gamepad2.left_trigger;
            shoter = gamepad2.left_stick_y;

            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                forward = -forward;
                side = -side;
                rotation = -rotation;
            }


            leftFront.setPower((forward - rotation - side) * SPEEDFACTOR);
            leftBack.setPower((forward - rotation + side)* SPEEDFACTOR);
            rightFront.setPower((forward + rotation + side)* SPEEDFACTOR);
            rightBack.setPower((forward + rotation - side)* SPEEDFACTOR);

            intake.setPower((intaker - intakel));
            shooter.setPower((shoter)*SPEEDSHOOT );


            sleep(50);

        }
    }
}

