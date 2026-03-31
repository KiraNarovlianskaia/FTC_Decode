package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Servos_Pattern_Red {

    private Servo servoL, servoM, servoR;

    private double servoPush = 0;
    private double servoOpen = 1;

    private ElapsedTime timer = new ElapsedTime();

    private int shootState = 0;
    private boolean shooting = false;

    private int shootId = 0;       // ID текущего паттерна выстрела
    private Servo[] shootOrder;


    public void init(HardwareMap hwMap) {
        servoL = hwMap.get(Servo.class, "servo_left");
        servoM = hwMap.get(Servo.class, "servo_mid");
        servoR = hwMap.get(Servo.class, "servo_right");

        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoPush);
    }

    // 🔥 Запуск последовательной стрельбы
    public void startShooting(int shootId, int variant) {
        this.shootId = shootId;
        shooting = true;
        shootState = 0;
        timer.reset();

        // пример: определяем порядок серв в зависимости от варианта
        switch (variant) {
            case 1:
                shootOrder = new Servo[]{servoR, servoM, servoL};
                break;
            case 2:
                shootOrder = new Servo[]{servoM, servoL, servoR};
                break;
            case 3:
                shootOrder = new Servo[]{servoL, servoM, servoR};
                break;
            case 4:
                shootOrder = new Servo[]{servoM, servoR, servoL};
                break;
            case 5:
                shootOrder = new Servo[]{servoL, servoR, servoM};
                break;
            default:
                shootOrder = new Servo[]{servoR, servoM, servoL};
                break;
        }
    }

    // ОБЯЗАТЕЛЬНО вызывать в loop()
    public void update() {
        if (!shooting) return;

        // если shootState меньше длины массива, продолжаем стрелять
        if (shootState < shootOrder.length) {
            // либо первый шаг, либо таймер прошёл
            if (shootState == 0 || timer.milliseconds() >= 1000) {
                if (shootOrder[shootState] == servoR){
                    shootOrder[shootState].setPosition(servoOpen);
                }
                else {
                    shootOrder[shootState].setPosition(servoPush);
                }// поднимаем текущий серво
                timer.reset();
                shootState++; // переходим к следующему
            }
        } else {
            shooting = false; // закончили все выстрелы
        }

    }

    public void closeAll() {
        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoPush);
    }
}