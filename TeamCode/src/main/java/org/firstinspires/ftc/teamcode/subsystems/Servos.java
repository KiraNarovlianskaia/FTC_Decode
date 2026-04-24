package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class Servos {

    // ================= ENUM =================
    public enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    // ================= HARDWARE =================
    private Servo servoL, servoM, servoR;
    private ColorSensor sensorL, sensorM, sensorR;

    private final double servoPush = 0;
    private final double servoOpen = 0.5;

    private ElapsedTime timer = new ElapsedTime();

    // ================= ЛОГИКА =================
    private boolean shooting = false;

    private List<BallColor> pattern = new ArrayList<>();
    private List<BallColor> history = new ArrayList<>();
    private List<BallColor> shootSequence = new ArrayList<>();

    private Servo[] servos = new Servo[3];
    private boolean[] used = new boolean[3];

    private int currentIndex = 0;

    // ================= INIT =================
    public void init(HardwareMap hwMap) {
        servoL = hwMap.get(Servo.class, "servo_left");
        servoM = hwMap.get(Servo.class, "servo_mid");
        servoR = hwMap.get(Servo.class, "servo_right");

        // Даже если датчики не подключены, оставляем инициализацию,
        // чтобы код не вылетал с ошибкой NullPointerException
        try {
            sensorL = hwMap.get(ColorSensor.class, "ball_color_left");
            sensorM = hwMap.get(ColorSensor.class, "ball_color_mid");
            sensorR = hwMap.get(ColorSensor.class, "ball_color_right");
        } catch (Exception e) {
            // Если датчиков физически нет в конфигурации, просто игнорируем
        }

        servos[0] = servoL;
        servos[1] = servoM;
        servos[2] = servoR;

        closeAll();
    }

    // ================= ЗАПУСК =================
    public void startShooting(List<BallColor> pattern) {
        if (pattern == null || pattern.isEmpty()) return;

        this.pattern = pattern;
        shootSequence.clear();

        // Сброс состояния использования серв
        for (int i = 0; i < 3; i++) used[i] = false;

        // Формируем план на 3 выстрела (заглушка, так как датчиков нет)
        for (int i = 0; i < 3; i++) {
            shootSequence.add(BallColor.UNKNOWN);
        }

        shooting = true;
        currentIndex = 0;
        timer.reset();
    }

    // ================= UPDATE (БЕЗ ДАТЧИКОВ) =================
    public void update() {
        if (!shooting) return;

        // Интервал между выстрелами 700мс
        if (timer.milliseconds() < 700) return;

        if (currentIndex < shootSequence.size()) {

            int foundIndex = -1;

            // Просто ищем следующую по порядку серву, которая еще не стреляла
            for (int i = 0; i < 3; i++) {
                if (!used[i]) {
                    foundIndex = i;
                    break;
                }
            }

            // Если нашли серву — стреляем
            if (foundIndex != -1) {
                // Логика движения
                if (foundIndex == 2) {
                    // Для правой сервы используем твою логику (0.0)
                    servos[foundIndex].setPosition(servoPush+1);
                } else {
                    // Для левой и средней (0.5)
                    servos[foundIndex].setPosition(servoOpen);
                }

                used[foundIndex] = true;
                currentIndex++;
                timer.reset();
            } else {
                shooting = false;
            }
        } else {
            shooting = false;
        }
    }

    // ================= СЕРВО УПРАВЛЕНИЕ =================
    public void closeAll() {
        // Устанавливаем сервы в исходное положение
        servoL.setPosition(servoPush); // 0.0
        servoM.setPosition(servoPush); // 0.0
        servoR.setPosition(servoOpen); // 0.5
    }

    public void resetHistory() {
        history.clear();
    }

    public boolean isShooting() {
        return shooting;
    }

    // Метод getColor оставлен для совместимости, но не используется в update
    private BallColor getColor(ColorSensor s) {
        return BallColor.UNKNOWN;
    }

    // ================= ТЕЛЕМЕТРИЯ =================
    public void addTelemetryData(com.qualcomm.robotcore.hardware.Gamepad gamepad, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("--- SHOOTING (NO SENSORS MODE) ---");
        telemetry.addData("Status", shooting ? "SHOOTING" : "IDLE");
        telemetry.addData("Current Index", currentIndex);
        telemetry.addData("Timer (ms)", (int)timer.milliseconds());
    }
}