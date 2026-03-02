package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    private DcMotor shooterL, shooterM, shooterR;
    private Servo servoL, servoM, servoR;
    private ElapsedTime stateTimer = new ElapsedTime();

    private enum ShooterState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        COOLDOWN
    }

    private ShooterState shooterState;

    //-------------GATE CONSTANTS------------
    private double servo_init = 0.0;
    private double servo_shoot = 0.45;
    private double servo_open_time = 0.5;

    //-------------FLYWHEEL CONSTANTS---------
    private double flywheelVelocity = 0.7;
    private double flywheel_max_spinup_time = 2;

    //-------------COOLDOWN CONSTANTS---------
    private double cooldown_time = 1;
    private double cooldown_vel = -0.5;

    public void init(HardwareMap hwMap) {
        servoL = hwMap.get(Servo.class, "servo_left");
        servoM = hwMap.get(Servo.class, "servo_mid");
        servoR = hwMap.get(Servo.class, "servo_right");

        shooterL = hwMap.get(DcMotor.class, "shooter_left");
        shooterM = hwMap.get(DcMotor.class, "shooter_mid");
        shooterR = hwMap.get(DcMotor.class, "shooter_right");

        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterM.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterState = ShooterState.IDLE;

        servoL.setPosition(servo_init);
        servoM.setPosition(servo_init);
        servoR.setPosition(servo_init);

        shooterL.setPower(0);
        shooterM.setPower(0);
        shooterR.setPower(0);
    }

    public void shoot() {
        if (shooterState == ShooterState.IDLE) {
            shooterState = ShooterState.SPIN_UP;
            stateTimer.reset();
            shooterL.setPower(flywheelVelocity);
            shooterM.setPower(flywheelVelocity);
            shooterR.setPower(flywheelVelocity);
        }
    }

    public void update() {

        switch (shooterState) {

            case IDLE:
                shooterL.setPower(0); // защита
                shooterM.setPower(0);
                shooterR.setPower(0);
                break;

            case SPIN_UP:
                if (stateTimer.seconds() > flywheel_max_spinup_time) {
                    servoL.setPosition(servo_shoot);
                    servoM.setPosition(servo_shoot);
                    servoR.setPosition(servo_shoot);
                    stateTimer.reset();
                    shooterState = ShooterState.LAUNCH;
                }
                break;

            case LAUNCH:
                if (stateTimer.seconds() > servo_open_time) {
                    servoL.setPosition(servo_init);
                    servoM.setPosition(servo_init);
                    servoR.setPosition(servo_init);

                    shooterL.setPower(cooldown_vel);
                    shooterM.setPower(cooldown_vel);
                    shooterR.setPower(cooldown_vel);
                    stateTimer.reset();
                    shooterState = ShooterState.COOLDOWN;
                }
                break;

            case COOLDOWN:
                if (stateTimer.seconds() > cooldown_time) {
                    shooterL.setPower(0);
                    shooterM.setPower(0);
                    shooterR.setPower(0);
                    shooterState = ShooterState.IDLE;
                }
                break;
        }
    }

    public boolean isBusy() {
        return shooterState != ShooterState.IDLE;
    }
}
