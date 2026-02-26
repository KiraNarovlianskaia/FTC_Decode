package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeMotor;

    // --------- CONSTANTS ----------
    private double intakePower = 1;   // мощность всасывания

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0); // безопасность
    }

    // Запуск мотора
    public void start() {
        intakeMotor.setPower(intakePower);
    }

    // Остановка мотора
    public void stop() {
        intakeMotor.setPower(0);
    }
}
