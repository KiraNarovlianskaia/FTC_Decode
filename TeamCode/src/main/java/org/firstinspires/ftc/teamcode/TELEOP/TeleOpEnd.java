package org.firstinspires.ftc.teamcode.TELEOP;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@Configurable
@TeleOp(name="TeleOpEnd")

public class TeleOpEnd extends LinearOpMode {
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor intake;
    DcMotor shooter_left;
    DcMotor shooter_right;
    DcMotor shooter_mid;
    Servo servo_left;
    Servo servo_right;
    Servo servo_mid;


    // Servo position vars
    static final double servoPush = 0.45;
    static final double servoOpen = 0;
    static final double servoReady = 0.18;


    // Speed vars
    static final double SPEEDFACTOR = 0.55;
    static final double intakeSpeed = 0.9;
    static double shootingSpeed = 0.85;


    // Variables for color detection
    String ball_left_color = "None";

    String ball_mid_color = "None";
    String ball_right_color = "None";



    public void runOpMode(){


        double forward;
        double rotation;
        double side;
        double intaker;
        double shoter_left ;
        double shoter_right ;
        double shoter_mid ;


        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter_left = hardwareMap.get(DcMotor.class, "shooter_left");
        shooter_mid = hardwareMap.get(DcMotor.class, "shooter_mid");
        shooter_right = hardwareMap.get(DcMotor.class, "shooter_right");



        servo_left = hardwareMap.get(Servo.class, "servo_left");
        servo_right = hardwareMap.get(Servo.class, "servo_right");
        servo_mid = hardwareMap.get(Servo.class, "servo_mid");



        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ball_color_left");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "ball_color_right");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "ball_color_mid");



        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter_right.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        servo_left.setPosition(servoOpen);
        servo_right.setPosition(servoOpen);
        servo_mid.setPosition(servoOpen);


        while (opModeIsActive()) {


// Open and close left servo
            if (gamepad1.xWasPressed()) {
                servo_left.setPosition(servoPush);
                sleep(400);
                servo_left.setPosition(servoOpen);
            }

// Open and close right servo
            if (gamepad1.bWasPressed()) {
                servo_right.setPosition(servoPush);
                sleep(400);
                servo_right.setPosition(servoOpen);
            }

            if (gamepad1.yWasPressed()) {
                servo_mid.setPosition(servoPush);
                sleep(400);
                servo_mid.setPosition(servoOpen);
            }

// Open and close both servos
            if (gamepad1.aWasPressed()) {
                servo_left.setPosition(servoPush);
                servo_right.setPosition(servoPush);
                servo_mid.setPosition(servoPush);
                sleep(400);
                servo_right.setPosition(servoOpen);
                servo_left.setPosition(servoOpen);
                servo_mid.setPosition(servoOpen);
            }


            shoter_left = gamepad2.left_trigger;
            shoter_right = gamepad2.right_trigger;
            shoter_mid = gamepad2.right_stick_y;




            forward = gamepad1.left_stick_y;
            side = gamepad1.right_trigger-gamepad1.left_trigger;
            rotation = gamepad1.right_stick_x;
            intaker = gamepad2.left_stick_y;




            leftFront.setPower((forward - rotation - side) * SPEEDFACTOR);
            leftBack.setPower((forward - rotation + side)* SPEEDFACTOR);
            rightFront.setPower((forward + rotation + side)* SPEEDFACTOR);
            rightBack.setPower((forward + rotation - side)* SPEEDFACTOR);

            intake.setPower((intaker));
            shooter_right.setPower((shoter_right)*shootingSpeed );
            shooter_left.setPower((shoter_left)*shootingSpeed);
            shooter_mid.setPower((shoter_mid)*shootingSpeed);

// Normalize the colors
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
            NormalizedRGBA colors3 = colorSensor3.getNormalizedColors();


// LEFT BALL COLOR
            if (colors.alpha < 0.3 /* > colors.blue && colors.alpha > colors.green */ ) {
                ball_left_color = "None";
            } else if (colors.blue > colors.green) {
                ball_left_color = "Purple";
            } else {
                ball_left_color = "Green";
            }

// RIGHT BALL COLOR
            if (colors2.alpha < 0.3 /* > colors2.blue && colors2.alpha > colors2.green */) {
                ball_right_color = "None";
            } else if (colors2.blue > colors2.green) {
                ball_right_color = "Purple";
            } else {
                ball_right_color = "Green";
            }
// Mid ball color
            if (colors3.alpha < 0.3 /* > colors3.blue && colors3.alpha > colors3.green */ ) {
                ball_mid_color = "None";
            } else if (colors3.blue > colors3.green) {
                ball_mid_color = "Purple";
            } else {
                ball_mid_color = "Green";
            }


            telemetry.addData("Left ball: ", ball_left_color);
            telemetry.addData("Right ball: ", ball_right_color);
            telemetry.addData("Medium ball", ball_mid_color);
            telemetry.addData("Shooter speed: ", shootingSpeed);
            telemetry.update();


            if (ball_left_color != "None") {
                servo_left.setPosition(servoReady);
            }
            if (ball_right_color != "None") {
                servo_right.setPosition(servoReady);
            }
            if (ball_mid_color != "None") {
                servo_mid.setPosition(servoReady);
            }


            if (gamepad2.dpad_down) {
                shootingSpeed -= 0.01;
                sleep(50);
            }

            if (gamepad2.dpad_up) {
                shootingSpeed += 0.01;
                sleep(50);
            }


            sleep(50);

        }
    }
}
