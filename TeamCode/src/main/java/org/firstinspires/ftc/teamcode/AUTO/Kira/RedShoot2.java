package org.firstinspires.ftc.teamcode.AUTO.Kira;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "RED SingleShot", group = "Paths")
public class RedShoot2 extends OpMode {

    private Follower follower;
    private Shooter shooter;

    private enum State {
        DRIVE_TO_SHOOT,
        SHOOT,
        DONE
    }

    private State state;

    // ===== Позиции =====
    private final Pose startPose = new Pose(120.126, 118.791, Math.toRadians(-90));
    private final Pose shootPose = new Pose(96.689, 94.862, Math.toRadians(-135));

    private PathChain startToShoot;

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
        startToShoot = buildLine(startPose, shootPose);

        state = State.DRIVE_TO_SHOOT;
    }

    @Override
    public void loop() {

        // ОБЯЗАТЕЛЬНО обновляем
        follower.update();
        shooter.update();

        switch (state) {

            // ==========================================
            case DRIVE_TO_SHOOT:

                if (!pathStarted) {
                    follower.followPath(startToShoot, 0.6, true);
                    pathStarted = true;
                }

                // Раскрутка во время движения
                shooter.spinUp();

                // Когда приехали
                if (!follower.isBusy()) {
                    state = State.SHOOT;
                }

                break;

            // ==========================================
            case SHOOT:

                // Если вдруг не раскрутился — продолжаем
                shooter.spinUp();

                // Когда готов — стреляем
                if (shooter.isReady()) {
                    shooter.fire();
                }

                // Когда полностью завершил цикл
                if (shooter.isFinished()) {
                    state = State.DONE;
                }

                break;

            // ==========================================
            case DONE:

                // Дополнительная защита
                shooter.stop();
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.addData("Shooter Ready", shooter.isReady());
        telemetry.addData("Shooter Finished", shooter.isFinished());
        telemetry.update();
    }
}
