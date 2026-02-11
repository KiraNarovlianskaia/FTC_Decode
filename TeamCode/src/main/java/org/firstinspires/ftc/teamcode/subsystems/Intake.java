package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Intake {

    private DcMotor intake;
    private final Timer timer = new Timer();

    private enum State {
        IDLE,
        RUNNING
    }

    private State state = State.IDLE;

    public Intake(DcMotor intake) {
        this.intake = intake;
        intake.setPower(0);
    }

    // Запуск intake (можно во время движения)
    public void start() {
        if (state == State.IDLE) {
            timer.resetTimer();
            intake.setPower(Constants.intake_power);
            state = State.RUNNING;
        }
    }

    // ОБЯЗАТЕЛЬНО вызывать в loop
    public void update() {
        if (state == State.RUNNING) {
            if (timer.getElapsedTimeSeconds() >= 1.5) {
                intake.setPower(0);
                state = State.IDLE;
            }
        }
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public void stop() {
        intake.setPower(0);
        state = State.IDLE;
    }
}
