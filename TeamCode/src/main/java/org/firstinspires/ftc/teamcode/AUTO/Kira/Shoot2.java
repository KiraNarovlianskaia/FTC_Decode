package org.firstinspires.ftc.teamcode.AUTO.Kira;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "SingleShot", group = "Paths")
public class Shoot2 extends OpMode {

    private Follower follower;
    private Shooter shooter;

    private enum State {
        DRIVE_TO_SHOOT,
        SHOOT,
        DONE
    }

    private State state;

    // Позиции
    private final Pose startPose = new Pose(24.508, 119.077, Math.toRadians(-90));
    private final Pose shootPose = new Pose(48.594, 94.576, Math.toRadians(-45));
    private final Pose collectCurve1 = new Pose(23.99, 88.429, Math.toRadians(-90));

    private final Pose control1 = new Pose(22.27, 119.848, 0);
    private PathChain start_to_shoot;

    private boolean pathStarted = false;

    private PathChain buildLine(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        shooter = new Shooter(
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, Constants.shooter),
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, Constants.servoL),
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, Constants.servoR)
        );

        follower.setPose(startPose);

        start_to_shoot = buildLine(startPose, shootPose);

        state = State.DRIVE_TO_SHOOT;
    }

    @Override
    public void loop() {

        follower.update();
        shooter.update();

        switch (state) {

            case DRIVE_TO_SHOOT:

                // начинаем движение
                if (!pathStarted) {
                    follower.followPath(start_to_shoot, 0.6, true);
                    pathStarted = true;
                }

                // раскручиваем во время движения
                shooter.spinUp();

                // когда доехали — переходим к стрельбе
                if (!follower.isBusy()) {
                    state = State.SHOOT;
                }

                break;

            case SHOOT:

                // продолжаем держать раскрутку
                shooter.spinUp();

                // даём команду на выстрел
                shooter.fire();

                // ждём пока shooter полностью закончит (вернётся в IDLE)
                if (!shooter.isBusy()) {
                    state = State.DONE;
                }

                break;

            case DONE:

                shooter.stop();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.update();
    }
}
