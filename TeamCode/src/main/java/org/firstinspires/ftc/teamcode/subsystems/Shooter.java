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
        FIRING,
        COOLDOWN,
        BRAKING,
        FINISHED
    }

    private State state = State.IDLE;

    // Тайминги
    private static final double SERVO_TIME = 0.5;       // сервы вверх
    private static final double EXTRA_SPIN_TIME = 1.0;  // мотор ещё крутится после возврата серв
    private static final double BRAKE_POWER = -1;     // реверс для торможения
    private static final double BRAKE_TIME = 0.2;       // длительность реверса

    public Shooter(DcMotor shooter, Servo servoL, Servo servoR) {
        this.shooter = shooter;
        this.servoL = servoL;
        this.servoR = servoR;

        shooter.setPower(0);
        resetServos();
    }

    // Раскрутка
    public void spinUp() {
        if (state == State.IDLE) {
            shooter.setPower(Constants.shooter_power);
            timer.resetTimer();
            state = State.SPINNING;
        }
    }

    // Выстрел
    public void fire() {
        if (state == State.SPINNING && timer.getElapsedTimeSeconds() >= 3.0) {
            shootServos();
            timer.resetTimer();
            state = State.FIRING;
        }
    }

    // ОБЯЗАТЕЛЬНО в loop
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
                // ничего не делаем
                break;
        }
    }

    public boolean isReady() {
        return state == State.SPINNING && timer.getElapsedTimeSeconds() >= 3.0;
    }

    public boolean isBusy() {
        return state == State.SPINNING || state == State.FIRING || state == State.COOLDOWN || state == State.BRAKING;
    }

    public boolean isFinished() {
        return state == State.FINISHED;
    }

    public void stop() {
        shooter.setPower(0);
        resetServos();
        state = State.IDLE;
    }

    private void shootServos() {
        servoL.setPosition(Constants.servo_shoot);
        servoR.setPosition(Constants.servo_shoot);
    }

    private void resetServos() {
        servoL.setPosition(Constants.servo_init);
        servoR.setPosition(Constants.servo_init);
    }
}
