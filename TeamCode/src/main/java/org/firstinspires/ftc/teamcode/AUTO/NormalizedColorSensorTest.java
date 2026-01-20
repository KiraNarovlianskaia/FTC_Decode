package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Normalized Color Sensor Test")
public class NormalizedColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Vars for ball colors
        String ball1;
        String ball2;
        String ball3;

        // Map the colors sensors on hardware
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ball_color");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "ball_color2");

        waitForStart();

        while (opModeIsActive()) {
            // Normalize the colors
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();

            // Ball 1
            if (colors.alpha > colors.blue && colors.alpha > colors.green) {
                ball1 = "No Color";
            } else if (colors.blue > colors.green) {
                ball1 = "Purple";
            } else {
                ball1 = "Green";
            }

            // Ball 2
            if (colors2.alpha > colors2.blue && colors2.alpha > colors2.green) {
                ball2 = "No Color";
            } else if (colors2.blue > colors2.green) {
                ball2 = "Purple";
            } else {
                ball2 = "Green";
            }

            // Ball 3
            if (ball1.equals("Purple") && ball2.equals("Purple")) {
                ball3 = "Green";
            } else if ((ball1.equals("Purple") && ball2.equals("Green")) || (ball1.equals("Green") && ball2.equals("Purple"))) {
                ball3 = "Purple";
            } else {
                ball3 = "No Color";
            }

            // Print values in telemetry
            telemetry.addData("Hole 1: ", ball1);
            telemetry.addData("Hole 2: ", ball2);
            telemetry.addData("Hole 3: ", ball3);
            telemetry.update();
        }
    }
}