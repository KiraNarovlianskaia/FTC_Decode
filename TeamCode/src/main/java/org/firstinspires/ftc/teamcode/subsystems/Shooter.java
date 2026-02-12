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
        IDLE,       // ничего не делает
        SPINNING,   // раскрутка мотора
        FIRING,     // выстрел с серво
        COOLDOWN,   // мотор ещё крутится после возврата серво
        BRAKING,    // реверс для остановки
        FINISHED    // закончено, готов к следующему циклу
    }

    private State state = State.IDLE;

    // Тайминги и параметры
    private static final double SERVO_TIME = 0.5;        // время работы серво вверх
    private static final double EXTRA_SPIN_TIME = 1.0;   // дополнительное время вращения после выстрела
    private static final double BRAKE_POWER = -1.0;      // реверс для торможения
    private static final double BRAKE_TIME = 0.2;        // длительность реверса

    public Shooter(DcMotor shooter, Servo servoL, Servo servoR) {
        this.shooter = shooter;
        this.servoL = servoL;
        this.servoR = servoR;

        shooter.setPower(0);
        resetServos();
    }

    // Запуск мотора (можно параллельно с движением)
    public void spinUp() {
        if (state == State.IDLE) {
            shooter.setPower(Constants.shooter_power);
            timer.resetTimer();
            state = State.SPINNING;
        }
    }

    // Выстрел (будет выполняться только после достаточной раскрутки)
    public void fire() {
        if (state == State.SPINNING && timer.getElapsedTimeSeconds() >= 3.0) {
            shootServos();
            timer.resetTimer();
            state = State.FIRING;
        }
    }

    // Обновление состояния, ОБЯЗАТЕЛЬНО в loop
    public void update() {

        switch (state) {

            case FIRING:
                if (timer.getElapsedTimeSeconds() >= SERVO_TIME) {
                    resetServos();
                    timer.resetTimer();
                    state = State.COOLDOWN;
                }
                break;

            case COOLDOWN:
                if (timer.getElapsedTimeSeconds() >= EXTRA_SPIN_TIME) {
                    shooter.setPower(BRAKE_POWER);  // короткий реверс для торможения
                    timer.resetTimer();
                    state = State.BRAKING;
                }
                break;

            case BRAKING:
                if (timer.getElapsedTimeSeconds() >= BRAKE_TIME) {
                    shooter.setPower(0);
                    state = State.FINISHED;
                }
                break;

            case FINISHED:
                // ничего не делаем, ждём следующего цикла
                break;

            case SPINNING:
            case IDLE:
                // ничего не меняем, ждём вызова fire() или spinUp()
                break;
        }
    }

    // Проверка, готов ли к выстрелу
    public boolean isReady() {
        return state == State.SPINNING && timer.getElapsedTimeSeconds() >= 3.0;
    }

    // Проверка, занято ли устройство
    public boolean isBusy() {
        return state == State.SPINNING || state == State.FIRING || state == State.COOLDOWN || state == State.BRAKING;
    }

    // Проверка, завершена ли работа
    public boolean isFinished() {
        return state == State.FINISHED;
    }

    // Принудительная остановка и сброс
    public void stop() {
        shooter.setPower(0);
        resetServos();
        state = State.IDLE;
    }

    // Серво вверх для выстрела
    private void shootServos() {
        servoL.setPosition(Constants.servo_shoot);
        servoR.setPosition(Constants.servo_shoot);
    }

    // Серво вниз (инициализация)
    private void resetServos() {
        servoL.setPosition(Constants.servo_init);
        servoR.setPosition(Constants.servo_init);
    }
}
