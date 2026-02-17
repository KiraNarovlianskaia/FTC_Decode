package org.firstinspires.ftc.teamcode.FinalCodes.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name="TeleOp2Balls")

public class TeleOp3Balls extends LinearOpMode {
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;
    DcMotor shooter;
    Servo servoL;
    Servo servoR;


    // Servo position vars
    static final double servoPush = 0.45;
    static final double servoOpen = 0;
    static final double servoReady = 0.18;


    // Speed vars
    static final double SPEEDFACTOR = 0.55;
    static final double intakeSpeed = 0.9;
    static double shootingSpeed = 0.85;


    // Ball colors
    String ball_left_color = "None";
    String ball_middle_color = "None";
    String ball_right_color = "None";


    public void runOpMode(){


        double forward;
        double rotation;
        double side;
        double intakeR;
        double intakeL;
        double shoter;


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        servoL = hardwareMap.get(Servo.class, "left_servo");
        servoR = hardwareMap.get(Servo.class, "right_servo");

        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ball_color");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "ball_color2");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "ball_color3");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        servoL.setPosition(servoOpen);
        servoR.setPosition(servoOpen);


        while (opModeIsActive()) {


            // Open and close left servo
            if (gamepad2.xWasPressed()) {
                servoL.setPosition(servoPush);
                sleep(400);
                servoL.setPosition(servoOpen);
            }

            // Open and close right servo
            if (gamepad2.bWasPressed()) {
                servoR.setPosition(servoPush);
                sleep(400);
                servoR.setPosition(servoOpen);
            }

            // Open and close both servos
            if (gamepad2.yWasPressed()) {
                servoL.setPosition(servoPush);
                servoR.setPosition(servoPush);
                sleep(400);
                servoL.setPosition(servoOpen);
                servoR.setPosition(servoOpen);
            }


            forward = gamepad1.left_stick_y;
            side = gamepad1.left_stick_x;
            rotation = gamepad1.right_stick_x;
            intakeR = gamepad2.right_trigger;
            intakeL = gamepad2.left_trigger;
            shoter = gamepad2.left_stick_y;


            leftFront.setPower((forward - rotation - side) * SPEEDFACTOR);
            leftBack.setPower((forward - rotation + side)* SPEEDFACTOR);
            rightFront.setPower((forward + rotation + side)* SPEEDFACTOR);
            rightBack.setPower((forward + rotation - side)* SPEEDFACTOR);

            intake.setPower((intakeR - intakeL));
            shooter.setPower((shoter)*shootingSpeed );

            // Normalize the colors
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
            NormalizedRGBA colors3 = colorSensor2.getNormalizedColors();


            // LEFT BALL COLOR
            if (colors.alpha < 0.3 /* > colors.blue && colors.alpha > colors.green */ ) {
                ball_left_color = "None";
            } else if (colors.blue > colors.green) {
                ball_left_color = "Purple";
            } else {
                ball_left_color = "Green";
            }
            
            // RIGHT BALL COLOR
            if (colors3.alpha < 0.3 /* > colors3.blue && colors3.alpha > colors3.green */) {
                ball_right_color = "None";
            } else if (colors3.blue > colors3.green) {
                ball_right_color = "Purple";
            } else {
                ball_right_color = "Green";
            }

            // MIDDLE BALL COLOR
            if (colors2.alpha < 0.2 /* > colors2.blue && colors2.alpha > colors2.green */) {
                ball_middle_color = "None";
            } else if (colors2.blue > colors2.green) {
                ball_middle_color = "Purple";
            } else {
                ball_middle_color = "Green";
            }


            telemetry.addData("Left ball: ", ball_left_color);
            telemetry.addData("Middle ball: ", ball_middle_color);
            telemetry.addData("Right ball: ", ball_right_color);
            telemetry.addData("Shooter speed: ", shootingSpeed);
            telemetry.update();


            if (ball_left_color != "None") {
                servoR.setPosition(servoReady);
            } else {
                servoR.setPosition(servoOpen);
            }
            if (ball_right_color != "None") {
                servoL.setPosition(servoReady);
            } else {
                servoL.setPosition(servoOpen);
            }


            if (gamepad1.left_bumper) {
                shootingSpeed -= 0.01;
                sleep(50);
            }
            if (gamepad1.right_bumper) {
                shootingSpeed += 0.01;
                sleep(50);
            }


            sleep(50);
        }
    }
}

