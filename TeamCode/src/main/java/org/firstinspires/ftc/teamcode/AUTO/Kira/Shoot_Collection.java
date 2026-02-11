package org.firstinspires.ftc.teamcode.AUTO.Kira;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "Shoot + Collect", group = "Paths")
public class Shoot_Collection extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Timer timer = new Timer();

    private enum State {
        DRIVE_TO_SHOOT,
        SHOOT,
        DRIVE_TO_COLLECT,
        INTAKE_FINISH,
        DONE
    }

    private State state;

    // Позиции
    private final Pose startPose = new Pose(24.508, 119.077, Math.toRadians(-90));
    private final Pose shootPose = new Pose(48.594, 94.576, Math.toRadians(-45));

    private final Pose collectCurve1 = new Pose(24.333, 89.033, Math.toRadians(-90));
    private final Pose control1 = new Pose(24.56, 118.989, 0);

    private PathChain start_to_shoot;
    private PathChain shoot_to_collect;

    private boolean pathStarted = false;

    private PathChain buildLine(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    private PathChain buildCurve(Pose from, Pose control, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(from, control, to))
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

        intake = new Intake(
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, Constants.intake)
        );

        follower.setPose(startPose);

        start_to_shoot = buildLine(startPose, shootPose);
        shoot_to_collect = buildCurve(shootPose, control1, collectCurve1);

        state = State.DRIVE_TO_SHOOT;
    }

    @Override
    public void loop() {

        follower.update();
        shooter.update();

        switch (state) {

            case DRIVE_TO_SHOOT:

                if (!pathStarted) {
                    follower.followPath(start_to_shoot, 0.6, true);
                    pathStarted = true;
                }

                shooter.spinUp();

                if (!follower.isBusy()) {
                    state = State.SHOOT;
                }

                break;

            case SHOOT:

                shooter.spinUp();
                shooter.fire();

                if (!shooter.isBusy()) {
                    pathStarted = false;
                    state = State.DRIVE_TO_COLLECT;
                }

                break;

            case DRIVE_TO_COLLECT:

                if (!pathStarted) {
                    follower.followPath(shoot_to_collect, 0.6, true);
                    pathStarted = true;
                }

                intake.start();

                if (!follower.isBusy()) {
                    timer.resetTimer();      // запускаем таймер 3 сек
                    state = State.INTAKE_FINISH;
                }

                break;

            case INTAKE_FINISH:

                intake.start(); // продолжаем крутить

                if (timer.getElapsedTimeSeconds() >= 3.0) {
                    state = State.DONE;
                }

                break;

            case DONE:

                shooter.stop();
                intake.stop();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }
}
