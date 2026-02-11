package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Intake {

    private DcMotor intake;

    public Intake(DcMotor intake) {
        this.intake = intake;
        intake.setPower(0);
    }

    // Запуск — крутится постоянно
    public void start() {
        intake.setPower(Constants.intake_power);
    }

    // Теперь ничего не делает
    public void update() {
        // пусто
    }

    public void stop() {
        intake.setPower(0);
    }

    public boolean isBusy() {
        return false;
    }
}
