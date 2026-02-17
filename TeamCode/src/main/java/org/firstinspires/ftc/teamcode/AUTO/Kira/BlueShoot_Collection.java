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
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "BLUE Shoot + Collect", group = "Paths")
public class BlueShoot_Collection extends OpMode {

    private Follower follower;
//    private Shooter shooter;
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

    // ===== Позиции =====
    private final Pose startPose = new Pose(24.508, 119.077, Math.toRadians(-90));
    private final Pose shootPose = new Pose(48.594, 94.576, Math.toRadians(-45));
    private final Pose collectCurve1 = new Pose(24.333, 89.033, Math.toRadians(-90));
    private final Pose control1 = new Pose(24.56, 118.989, 0);

    private PathChain startToShoot;
    private PathChain shootToCollect;

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

//        shooter = new Shooter(
//                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, Constants.shooter),
//                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, Constants.servoL),
//                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, Constants.servoR)
//        );

        intake = new Intake(
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, Constants.intake)
        );

        follower.setPose(startPose);

        startToShoot = buildLine(startPose, shootPose);
        shootToCollect = buildCurve(shootPose, control1, collectCurve1);

        state = State.DRIVE_TO_SHOOT;
    }

    @Override
    public void loop() {

        follower.update();
//        shooter.update();

        switch (state) {

            // =====================================================
            case DRIVE_TO_SHOOT:

                if (!pathStarted) {
                    follower.followPath(startToShoot, 0.6, true);
                    pathStarted = true;
                }

                // Параллельно раскручиваем шутер
//                shooter.spinUp();

                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.DRIVE_TO_COLLECT;
                }

                break;

            // =====================================================


            // =====================================================
            case DRIVE_TO_COLLECT:

                if (!pathStarted) {
                    follower.followPath(shootToCollect, 0.6, true);
                    pathStarted = true;
                }

                // Интейк включаем сразу
                intake.start();

                // НЕ блокируем движение, кулдаун шутера продолжается
                // Ждём, чтобы он завершил кулдаун и торможение перед переходом к INTAKE_FINISH
//                if (!follower.isBusy() && shooter.isFinished()) {
//                    timer.resetTimer();
//                    state = State.INTAKE_FINISH;
//                }

                break;

            // =====================================================
            case INTAKE_FINISH:

                intake.start();

                if (timer.getElapsedTimeSeconds() >= 3.0) {
                    state = State.DONE;
                }

                break;

            // =====================================================
            case DONE:

                shooter.stop();
                intake.stop();
                break;
        }

        // ===== Телеметрия =====
        telemetry.addData("State", state);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.addData("Shooter Ready", shooter.isReady());
        telemetry.addData("Shooter Finished", shooter.isFinished());
        telemetry.update();
    }
}
