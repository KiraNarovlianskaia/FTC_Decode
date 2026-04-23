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
    private final double servoOpen = 1;

    private ElapsedTime timer = new ElapsedTime();

    // ================= ЛОГИКА =================
    private boolean shooting = false;

    private List<BallColor> pattern = new ArrayList<>();
    private List<BallColor> history = new ArrayList<>();
    private List<BallColor> shootSequence = new ArrayList<>();

    private BallColor[] currentBalls = new BallColor[3];
    private Servo[] servos = new Servo[3];
    private boolean[] used = new boolean[3];

    private int currentIndex = 0;

    // ================= INIT =================
    public void init(HardwareMap hwMap) {
        servoL = hwMap.get(Servo.class, "servo_left");
        servoM = hwMap.get(Servo.class, "servo_mid");
        servoR = hwMap.get(Servo.class, "servo_right");

        sensorL = hwMap.get(ColorSensor.class, "ball_color_left");
        sensorM = hwMap.get(ColorSensor.class, "ball_color_mid");
        sensorR = hwMap.get(ColorSensor.class, "ball_color_right");

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

        readSensors();

        // Сброс состояния использования серв
        for (int i = 0; i < 3; i++) used[i] = false;

        // Формируем план на текущую серию (3 выстрела)
        int ballsToShoot = 3;

        for (int i = 0; i < ballsToShoot; i++) {
            // Определяем, какой шар ДОЛЖЕН быть следующим по глобальному паттерну
            int patternIndex = (history.size() + i) % pattern.size();
            BallColor target = pattern.get(patternIndex);
            shootSequence.add(target);
        }

        shooting = true;
        currentIndex = 0;
        timer.reset();
    }

    // ================= UPDATE =================
    public void update() {
        if (!shooting) return;

        // Интервал между выстрелами 700мс (можно настроить)
        if (timer.milliseconds() < 700) return;

        if (currentIndex < shootSequence.size()) {
            readSensors();
            BallColor target = shootSequence.get(currentIndex);

            int foundIndex = -1;

            // ШАГ 1: Ищем идеальное совпадение по цвету
            for (int i = 0; i < 3; i++) {
                if (!used[i] && currentBalls[i] == target && currentBalls[i] != BallColor.UNKNOWN) {
                    foundIndex = i;
                    break;
                }
            }

            // ШАГ 2: Если нужного цвета нет, ищем ЛЮБОЙ доступный шар
            if (foundIndex == -1) {
                for (int i = 0; i < 3; i++) {
                    if (!used[i] && currentBalls[i] != BallColor.UNKNOWN) {
                        foundIndex = i;
                        break;
                    }
                }
            }

            // ШАГ 3: Если шар найден (целевой или запасной), стреляем
            if (foundIndex != -1) {
                // Логика открытия (Right инвертирован согласно вашему коду)
                if (foundIndex == 2) {
                    servos[foundIndex].setPosition(servoOpen);
                } else {
                    servos[foundIndex].setPosition(servoPush);
                }

                // Запоминаем, что выстрелили (для истории паттерна)
                history.add(currentBalls[foundIndex]);
                used[foundIndex] = true;
                currentIndex++;
                timer.reset();
            } else {
                // Если датчики не видят ни одного шара в доступных слотах — выходим
                shooting = false;
            }
        } else {
            shooting = false;
        }
    }

    // ================= СЕНСОРЫ =================
    private void readSensors() {
        currentBalls[0] = getColor(sensorL);
        currentBalls[1] = getColor(sensorM);
        currentBalls[2] = getColor(sensorR);
    }

    private BallColor getColor(ColorSensor s) {
        int r = s.red();
        int g = s.green();
        int b = s.blue();

        // Порог "пустоты"
        if (g < 130 && b < 130 && r < 130) return BallColor.UNKNOWN;

        // Зеленый: G доминирует над R и B
        if (g > r * 1.5 && g > b) {
            return BallColor.GREEN;
        }

        // Фиолетовый: B доминирует над G и R
        if (b > g && b > r) {
            return BallColor.PURPLE;
        }

        return BallColor.UNKNOWN;
    }

    // ================= СЕРВО УПРАВЛЕНИЕ =================
    public void closeAll() {
        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoPush);
    }

    public void resetHistory() {
        history.clear();
    }

    public boolean isShooting() {
        return shooting;
    }

    // ================= ТЕЛЕМЕТРИЯ =================
    public void addTelemetryData(com.qualcomm.robotcore.hardware.Gamepad gamepad, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("--- Color Sensors ---");
        telemetry.addData("L", "%s (R:%d G:%d B:%d)", getColor(sensorL), sensorL.red(), sensorL.green(), sensorL.blue());
        telemetry.addData("M", "%s (R:%d G:%d B:%d)", getColor(sensorM), sensorM.red(), sensorM.green(), sensorM.blue());
        telemetry.addData("R", "%s (R:%d G:%d B:%d)", getColor(sensorR), sensorR.red(), sensorR.green(), sensorR.blue());

        telemetry.addLine("--- Shooting State ---");
        telemetry.addData("Status", shooting ? "SHOOTING" : "IDLE");
        telemetry.addData("Progress", "%d / %d", currentIndex, shootSequence.size());
        if (shooting && currentIndex < shootSequence.size()) {
            telemetry.addData("Searching For", shootSequence.get(currentIndex));
        }
        telemetry.addData("Total History", history.size());
    }
}