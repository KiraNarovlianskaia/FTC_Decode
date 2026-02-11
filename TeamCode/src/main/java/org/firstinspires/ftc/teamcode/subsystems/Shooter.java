package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Shooter {

    private DcMotor shooter;
    private Servo servoL, servoR;
    private Timer timer = new Timer();

    private enum State {
        IDLE,
        SPINNING,
        FIRING
    }

    private State state = State.IDLE;

    public Shooter(DcMotor shooter, Servo servoL, Servo servoR) {
        this.shooter = shooter;
        this.servoL = servoL;
        this.servoR = servoR;

        shooter.setPower(0);
        resetServos();
    }

    // Раскрутка (можно во время движения)
    public void spinUp() {
        if (state == State.IDLE) {
            timer.resetTimer();
            shooter.setPower(Constants.shooter_power);
            state = State.SPINNING;
        }
    }

    // Выстрел — ТОЛЬКО когда разрешили
    public void fire() {
        if (state == State.SPINNING && timer.getElapsedTimeSeconds() >= 3.0) {
            timer.resetTimer();
            shootServos();
            state = State.FIRING;
        }
    }

    // ОБЯЗАТЕЛЬНО вызывать в loop
    public void update() {
        if (state == State.FIRING) {
            if (timer.getElapsedTimeSeconds() >= 0.5) {
                resetServos();
                shooter.setPower(0);
                state = State.IDLE;
            }
        }
    }

    // Проверка, готов ли к выстрелу (раскрутился 3 секунды)
    public boolean isReady() {
        return state == State.SPINNING && timer.getElapsedTimeSeconds() >= 3.0;
    }

    // Проверка, занят ли shooter (SPINNING или FIRING)
    public boolean isBusy() {
        return state != State.IDLE;
    }

    // Полная остановка
    public void stop() {
        shooter.setPower(0);
        resetServos();
        state = State.IDLE;
    }

    // Запуск серв для выстрела
    private void shootServos() {
        servoL.setPosition(Constants.servo_shoot);
        servoR.setPosition(Constants.servo_shoot);
    }

    // Сброс серв в начальное положение
    private void resetServos() {
        servoL.setPosition(Constants.servo_init);
        servoR.setPosition(Constants.servo_init);
    }
}
