package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
@Disabled
@TeleOp(name = "Ball Detection Improved")
public class BallDetectionImproved extends LinearOpMode {

    @Override
    public void runOpMode() {

//        // Vars for ball colors
//        String ball_left;
//        String ball_mid;
//        String ball_right;

        // Map the color sensors on hardware
        NormalizedColorSensor colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "ball_color_left");
        NormalizedColorSensor colorSensorM = hardwareMap.get(NormalizedColorSensor.class, "ball_color_mid");
        NormalizedColorSensor colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "ball_color_right");

        waitForStart();

        while (opModeIsActive()) {

            // Read normalized colors
            NormalizedRGBA colorL = colorSensorL.getNormalizedColors();
            NormalizedRGBA colorM = colorSensorM.getNormalizedColors();
            NormalizedRGBA colorR = colorSensorR.getNormalizedColors();

            // Print all sensor values
            telemetry.addLine("Left Sensor");
            telemetry.addData("Red: ", colorL.red);
            telemetry.addData("Blue: ", colorL.blue);
            telemetry.addData("Green: ", colorL.green);
            telemetry.addData("Alpha: ", colorL.alpha);

            telemetry.addLine("Mid Sensor");
            telemetry.addData("Red: ", colorM.red);
            telemetry.addData("Blue: ", colorM.blue);
            telemetry.addData("Green: ", colorM.green);
            telemetry.addData("Alpha: ", colorM.alpha);

            telemetry.addLine("Right Sensor");
            telemetry.addData("Red: ", colorR.red);
            telemetry.addData("Blue: ", colorR.blue);
            telemetry.addData("Green: ", colorR.green);
            telemetry.addData("Alpha: ", colorR.alpha);

            telemetry.update();

//            // ---------------- Ball 1 ----------------
//            if (colors.blue > colors.green) { // Purple detection
//                ball1 = "Purple";
//            } else { // anything else with enough brightness is green
//                ball1 = "Green";
//            }
//
//            // ---------------- Ball 2 ----------------
//            if (colors2.blue > colors2.green) {
//                ball2 = "Purple";
//            } else {
//                ball2 = "Green";
//            }
//
//            // ---------------- Ball 3 ----------------
//            if (colors3.blue > colors3.green) {
//                ball3 = "Purple";
//            } else {
//                ball3 = "Green";
//            }
//
//            // Print values in telemetry
//            telemetry.addData("Hole 1: ", ball1);
//            telemetry.addData("Hole 2: ", ball2);
//            telemetry.addData("Hole 3: ", ball3);
//            telemetry.update();
        }
    }
}