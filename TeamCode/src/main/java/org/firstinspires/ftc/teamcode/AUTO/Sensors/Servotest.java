package org.firstinspires.ftc.teamcode.AUTO.Sensors;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="Servo Test", group="Robot")
public class Servotest extends LinearOpMode {

    private Servo servol;
    DcMotor shooter;
    static final double SPEEDFACTOR = 0.55;
    static double shootingSpeed = 0.85;


    boolean aBefore = false;
    boolean bBefore = false;
    boolean xBefore = false;
    boolean yBefore = false;
    boolean b1Before = false;
    boolean a1Before = false;
    boolean x1Before = false;
    boolean y1Before = false;
    @Override
    public void runOpMode() {
        servol = hardwareMap.get(Servo.class, "left_servo");
        servol.setPosition(0.5);


        double forward;
        double rotate;
        double side;

        double horizontal;
        double vertical;
        double shoter;

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad2.y && !yBefore) {
                yBefore = true;
                servol.setPosition(0.5);
            }
            yBefore = gamepad2.y;


            if (gamepad2.a && !aBefore) {
                aBefore = true;
                servol.setPosition(0);
            }
            aBefore = gamepad2.a;

            shoter = gamepad2.left_stick_y;
            shooter.setPower((shoter)*shootingSpeed );

        }
    }
}

