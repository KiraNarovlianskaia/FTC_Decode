package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServosThree {

    private Servo servoL, servoM, servoR;

    // Настройки позиций
    private final double servoPush = 0.0;
    private final double servoOpen = 0.5;

    public void init(HardwareMap hwMap) {
        servoL = hwMap.get(Servo.class, "servo_left");
        servoM = hwMap.get(Servo.class, "servo_mid");
        servoR = hwMap.get(Servo.class, "servo_right");

        closeAll();
    }

    /**
     * Мгновенно открывает все три сервы
     */
    public void shootAll() {
        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        // Используем servoPush для правой, так как она у тебя инвертирована
        servoR.setPosition(servoPush+1);
    }

    /**
     * Возвращает все сервы в исходное (закрытое) состояние
     */
    public void closeAll() {
        servoL.setPosition(servoPush);
        servoM.setPosition(servoPush);
        servoR.setPosition(servoOpen);
    }
}