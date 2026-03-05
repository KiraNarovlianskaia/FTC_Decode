package org.firstinspires.ftc.teamcode.archive.AUTO.Testing;

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

        // Map the color sensors on hardware
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ball_color_left");
        NormalizedColorSensor colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "ball_color_mid");
        NormalizedColorSensor colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "ball_color_right");

        waitForStart();

        while (opModeIsActive()) {

            // Read normalized colors
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
            NormalizedRGBA colors3 = colorSensor3.getNormalizedColors();

            // ---------------- Ball 1 ----------------
            if (colors.blue > colors.green) { // Purple detection (works for you)
                ball1 = "Purple";
            } else { // anything else with enough brightness is green
                ball1 = "Green";
            }

            // ---------------- Ball 2 ----------------
            if (colors2.blue > colors2.green) {
                ball2 = "Purple";
            } else {
                ball2 = "Green";
            }

            // ---------------- Ball 3 ----------------
            if (colors3.blue > colors3.green) {
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