package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class ServosSensors {

    public enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    private Servo servoL, servoM, servoR;
    private ColorSensor sensorL, sensorM, sensorR;

    private final double servoPush = 0;
    private final double servoOpen = 0.5;

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime stateTimer = new ElapsedTime(); // Таймер для защиты от застревания

    private boolean shooting = false;

    private List<BallColor> pattern = new ArrayList<>();
    private List<BallColor> history = new ArrayList<>();
    private List<BallColor> shootSequence = new ArrayList<>();

    private BallColor[] currentBalls = new BallColor[3];
    private Servo[] servos = new Servo[3];
    private boolean[] used = new boolean[3];

    private int currentIndex = 0;

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

    public void startShooting(List<BallColor> pattern) {
        if (pattern == null || pattern.isEmpty()) return;

        this.pattern = pattern;
        shootSequence.clear();
        for (int i = 0; i < 3; i++) used[i] = false;

        // Формируем желаемую последовательность цветов
        for (int i = 0; i < 3; i++) {
            int patternIndex = (history.size() + i) % pattern.size();
            shootSequence.add(pattern.get(patternIndex));
        }

        shooting = true;
        currentIndex = 0;
        shootTimer.reset();
        stateTimer.reset();
    }

    public void update() {
        if (!shooting) return;

        // Пауза между выстрелами (700мс)
        if (shootTimer.milliseconds() < 700) return;

        if (currentIndex < shootSequence.size()) {
            readSensors();
            BallColor target = shootSequence.get(currentIndex);
            int foundIndex = -1;

            // 1. Пытаемся найти нужный цвет
            for (int i = 0; i < 3; i++) {
                if (!used[i] && currentBalls[i] == target && currentBalls[i] != BallColor.UNKNOWN) {
                    foundIndex = i;
                    break;
                }
            }

            // 2. Если нужный цвет не найден, ищем ЛЮБОЙ шар (другого цвета)
            if (foundIndex == -1) {
                for (int i = 0; i < 3; i++) {
                    if (!used[i] && currentBalls[i] != BallColor.UNKNOWN) {
                        foundIndex = i;
                        break;
                    }
                }
            }

            // 3. ФОРС-МАЖОР: Если прошло 1.5 сек, а датчики молчат — стреляем любой свободной сервой
            if (foundIndex == -1 && stateTimer.seconds() > 1.5) {
                for (int i = 0; i < 3; i++) {
                    if (!used[i]) {
                        foundIndex = i;
                        break;
                    }
                }
            }

            // Если шар (или замена) найден — открываем серву
            if (foundIndex != -1) {
                if (foundIndex == 2) {
                    servos[foundIndex].setPosition(servoPush+0.5);
                } else {
                    servos[foundIndex].setPosition(servoOpen);
                }

                history.add(currentBalls[foundIndex]);
                used[foundIndex] = true;
                currentIndex++;
                shootTimer.reset();
                stateTimer.reset(); // Сбрасываем ожидание для следующего шара
            }
            // ВАЖНО: Мы НЕ пишем здесь shooting = false, чтобы робот продолжал попытки
        } else {
            shooting = false;
        }
    }

    private void readSensors() {
        currentBalls[0] = getColor(sensorL);
        currentBalls[1] = getColor(sensorM);
        currentBalls[2] = getColor(sensorR);
    }

    private BallColor getColor(ColorSensor s) {
        int r = s.red();
        int g = s.green();
        int b = s.blue();

        // Порог чувствительности (если датчики тупят, можно снизить до 50)
        if (g < 100 && b < 100 && r < 100) return BallColor.UNKNOWN;

        if (g > r * 1.4 && g > b) return BallColor.GREEN;
        if (b > g && b > r) return BallColor.PURPLE;

        return BallColor.UNKNOWN;
    }

    public void closeAll() {
        servoL.setPosition(servoPush);
        servoM.setPosition(servoPush);
        servoR.setPosition(servoOpen);
    }

    public boolean isShooting() {
        return shooting;
    }

    public void addTelemetryData(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("L", getColor(sensorL));
        telemetry.addData("M", getColor(sensorM));
        telemetry.addData("R", getColor(sensorR));
        telemetry.addData("Status", shooting ? "SHOOTING" : "IDLE");
        telemetry.addData("Waiting time", stateTimer.seconds());
    }
}