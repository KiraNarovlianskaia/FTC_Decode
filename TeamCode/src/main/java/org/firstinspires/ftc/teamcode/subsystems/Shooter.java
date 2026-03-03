package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private DcMotor shooterMotorL;
    private DcMotor shooterMotorM;
    private DcMotor shooterMotorR;

    // --------- CONSTANTS ----------
    private double shooterPower = 1;   // мощность всасывания

    public void init(HardwareMap hwMap) {
        shooterMotorL = hwMap.get(DcMotor.class, "shooter_left");
        shooterMotorM = hwMap.get(DcMotor.class, "shooter_mid");
        shooterMotorR = hwMap.get(DcMotor.class, "shooter_right");

        shooterMotorL.setDirection(DcMotor.Direction.FORWARD);
        shooterMotorM.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorR.setDirection(DcMotor.Direction.FORWARD);

        shooterMotorL.setPower(0); // безопасность
        shooterMotorM.setPower(0); // безопасность
        shooterMotorR.setPower(0); // безопасность
    }

    // Запуск мотора
    public void start() {
        shooterMotorL.setPower(shooterPower);
        shooterMotorM.setPower(shooterPower);
        shooterMotorR.setPower(shooterPower);
    }

    // Остановка мотора
    public void stop() {
        shooterMotorL.setPower(0);
        shooterMotorM.setPower(0);
        shooterMotorR.setPower(0);
    }
}
