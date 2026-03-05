package org.firstinspires.ftc.teamcode.AUTO.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Configurable
@Autonomous (name="ServoTestContinuous")

public class ServoTestContinuous extends LinearOpMode {

    CRServo servo;


    public void runOpMode() {
        servo = hardwareMap.get(CRServo.class, "servo");
        waitForStart();

        sleep(3000);
        servo.setPower(0.8);
        sleep(600);
        servo.setPower(0.005);
    }
}
