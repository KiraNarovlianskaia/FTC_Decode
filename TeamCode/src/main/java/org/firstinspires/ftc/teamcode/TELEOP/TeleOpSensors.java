package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "SENSOR_CALIBRATION", group = "Test")
public class TeleOpSensors extends OpMode {

    private ColorSensor sensorL, sensorM, sensorR;

    @Override
    public void init() {
        // Проверь, что имена в кавычках совпадают с конфигурацией на роботе!
        sensorL = hardwareMap.get(ColorSensor.class, "ball_color_left");
        sensorM = hardwareMap.get(ColorSensor.class, "ball_color_mid");
        sensorR = hardwareMap.get(ColorSensor.class, "ball_color_right");

        telemetry.addLine("Инициализация готова. Нажми START.");
    }

    @Override
    public void loop() {
        telemetry.addLine("=== ДАННЫЕ ДАТЧИКОВ (RGB) ===");

        // Левый датчик
        telemetry.addData("LEFT ", "R: %d | G: %d | B: %d", sensorL.red(), sensorL.green(), sensorL.blue());

        // Средний датчик
        telemetry.addData("MID  ", "R: %d | G: %d | B: %d", sensorM.red(), sensorM.green(), sensorM.blue());

        // Правый датчик
        telemetry.addData("RIGHT", "R: %d | G: %d | B: %d", sensorR.red(), sensorR.green(), sensorR.blue());

        telemetry.addLine("-----------------------------");
        telemetry.addLine("Поместите шар перед датчиком и запишите значения.");
        telemetry.update();
    }
}