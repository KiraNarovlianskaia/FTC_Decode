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
    private List<BallColor> history = new ArrayList<>(); // хранит ВСЕ выстреленные шары
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

        this.pattern = pattern;
        shootSequence.clear();

        readSensors();

        // сброс используемых серво только для этой серии
        for (int i = 0; i < 3; i++) used[i] = false;

        // формируем последовательность для текущей серии
        int ballsToShoot = 3; // количество шаров в этой серии

        for (int i = 0; i < ballsToShoot; i++) {
            BallColor target;

            // определяем индекс в паттерне с учётом истории всех выстрелов
            int patternIndex = history.size() % pattern.size();
            target = pattern.get(patternIndex);

            shootSequence.add(target);
            history.add(target); // добавляем в глобальную историю
        }

        shooting = true;
        currentIndex = 0;
        timer.reset();
    }

    // ================= UPDATE =================
    public void update() {
        if (!shooting) return;

        // Пауза между выстрелами 1.2 секунды
        if (timer.milliseconds() < 700) return;

        if (currentIndex < shootSequence.size()) {
            BallColor target = shootSequence.get(currentIndex);
            readSensors();

            int foundIndex = -1;
            for (int i = 0; i < 3; i++) {
                if (!used[i] && currentBalls[i] == target) {
                    foundIndex = i;
                    break;
                }
            }

            if (foundIndex != -1) {
                // Открываем серво
                if (foundIndex == 2) { // Right
                    servos[foundIndex].setPosition(servoOpen);
                } else { // Left & Mid
                    servos[foundIndex].setPosition(servoPush);
                }

                used[foundIndex] = true;
                currentIndex++;
                timer.reset();
            } else {
                // Если нужный шар не найден, мы просто НЕ сбрасываем таймер.
                // Робот будет проверять датчики каждый цикл, пока шар не появится.
            }
        } else {
            shooting = false;
            // Можно добавить задержку перед закрытием всех серв
        }
    }

    // ================= ОТКРЫТИЕ СЕРВО =================
    private void openServo(BallColor color) {

        readSensors(); // обновляем цвета перед выбором серво

        for (int i = 0; i < 3; i++) {

            if (!used[i] && currentBalls[i] == color) {

                // открываем нужное серво
                if (servos[i] == servoR) {
                    servos[i].setPosition(servoOpen);
                } else {
                    servos[i].setPosition(servoPush);
                }

                used[i] = true; // отмечаем серво как использованное
                return;
            }
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

        // Порог обнаружения (чтобы не реагировать на пустоту)
        // Твой максимум в пустоте был 122, ставим запас 130 для суммы или проверяем по G/B
        if (g < 130 && b < 130) return BallColor.UNKNOWN;

        // ЗЕЛЕНЫЙ: Зеленого должно быть ощутимо больше, чем красного
        // У тебя: G(271) vs R(84) — разница огромная.
        if (g > r * 1.5 && g > b) {
            return BallColor.GREEN;
        }

        // ФИОЛЕТОВЫЙ: Синего больше всего, и он больше зеленого
        // У тебя: B(251) vs G(171)
        if (b > g && b > r) {
            return BallColor.PURPLE;
        }

        return BallColor.UNKNOWN;
    }

    // ================= ПРОВЕРКА ПУСТО =================
    private boolean isEmpty() {
        for (BallColor c : currentBalls) {
            if (c != BallColor.UNKNOWN) return false;
        }
        return true;
    }

    // ================= СЕРВО =================
    public void closeAll() {
        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoPush);
    }

    // ================= ДОПОЛНИТЕЛЬНО =================
    public void resetHistory() {
        history.clear(); // можно сбросить историю между матчами
    }
    // Добавь это в класс Servos.java

    public void addTelemetryData(com.qualcomm.robotcore.hardware.Gamepad gamepad, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("--- Color Sensors ---");
        telemetry.addData("L (Left)", "R:%d G:%d B:%d -> %s", sensorL.red(), sensorL.green(), sensorL.blue(), getColor(sensorL));
        telemetry.addData("M (Mid) ", "R:%d G:%d B:%d -> %s", sensorM.red(), sensorM.green(), sensorM.blue(), getColor(sensorM));
        telemetry.addData("R (Right)", "R:%d G:%d B:%d -> %s", sensorR.red(), sensorR.green(), sensorR.blue(), getColor(sensorR));

        telemetry.addLine("--- Shooting State ---");
        telemetry.addData("Is Shooting", shooting);
        telemetry.addData("Current Index", currentIndex);
        if (!shootSequence.isEmpty() && currentIndex < shootSequence.size()) {
            telemetry.addData("Target Ball", shootSequence.get(currentIndex));
        }
    }
}