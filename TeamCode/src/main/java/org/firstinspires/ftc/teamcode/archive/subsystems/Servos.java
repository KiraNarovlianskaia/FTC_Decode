package org.firstinspires.ftc.teamcode.archive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    private Servo servoL, servoM, servoR;

    // --------- CONSTANTS ----------
    private double servoPush = 0;
    private double servoOpen = 1;

    public void init(HardwareMap hwMap) {
        servoL = hwMap.get(Servo.class, "servo_left");
        servoM = hwMap.get(Servo.class, "servo_mid");
        servoR = hwMap.get(Servo.class, "servo_right");

        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoOpen);
    }

    // Shoot servos
    public void servos_shoot() {
        servoL.setPosition(servoPush);
        servoM.setPosition(servoPush);
        servoR.setPosition(servoPush);
    }   

    //Close servos
    public void servos_close() {
        servoL.setPosition(servoOpen);
        servoM.setPosition(servoOpen);
        servoR.setPosition(servoOpen);
    }
}
