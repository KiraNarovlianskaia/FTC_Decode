package org.firstinspires.ftc.teamcode.AUTO.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_left");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "color_mid");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "color_right");

        waitForStart();

        while (opModeIsActive()) {
            // Normalize the colors
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
            NormalizedRGBA colors3 = colorSensor3.getNormalizedColors();

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
            if (colors3.alpha > colors3.blue && colors3.alpha > colors3.green) {
                ball3 = "No Color";
            } else if (colors3.blue > colors3.green) {
                ball3 = "Purple";
            } else {
                ball3 = "Green";
            }

            // Print values in telemetry
            telemetry.addData("Hole 1: ", ball1);
            telemetry.addData("Hole 2: ", ball2);
            telemetry.addData("Hole 3: ", ball3);
            telemetry.update();
        }
    }
}