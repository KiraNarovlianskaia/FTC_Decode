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

@Autonomous(name = "AutoBlue_9", group = "Paths")
public class allin_9 extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Timer timer = new Timer();

    private enum State {
        LINE_TO_SHOOT1, SHOOT1, CURVE_TO_COLLECT1, INTAKE1,
        LINE_TO_SHOOT2, SHOOT2,
        CURVE_TO_COLLECT2, LINE_COLLECT2,
        LINE_BACK_TO_SHOOT3, SHOOT3,
        CURVE_TO_COLLECT3, LINE_COLLECT3,
        LINE_TO_SHOOT4, SHOOT4,
        DONE
    }

    private State state;
    private boolean pathStarted = false;

    // Позиции
    private final Pose startPose = new Pose(24.508, 119.077, Math.toRadians(-90));
    private final Pose shootPose = new Pose(48.594, 94.576, Math.toRadians(135));
    private final Pose collectPose1 = new Pose(24.333, 89.033, Math.toRadians(-90));
    private final Pose collectCurve2 = new Pose(24.894, 74.719, Math.toRadians(-90));
    private final Pose collectPoint2 = new Pose(24.894, 65.848, Math.toRadians(-90));
    private final Pose collectCurve3 = new Pose(24.045, 53.170, Math.toRadians(-90));
    private final Pose collectPoint3 = new Pose(24.045, 40.079, Math.toRadians(-90));

    private final Pose control1 = new Pose(24.56, 118.989, 0);
    private final Pose control2 = new Pose(25.869, 84.504, 0);
    private final Pose control3 = new Pose(30.302, 78.164, 0);

    // Пути
    private PathChain start_shoot1;
    private PathChain shoot1_collect1;
    private PathChain collect1_shoot2;
    private PathChain shoot2_collectCurve2;
    private PathChain collectCurve2_collectPoint2;
    private PathChain collectPoint2_shoot3;
    private PathChain shoot3_collectCurve3;
    private PathChain collectCurve3_collectPoint3;
    private PathChain collectPoint3_shoot4;

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
        state = State.LINE_TO_SHOOT1;

        buildPaths();
    }

    private void buildPaths() {
        start_shoot1 = buildLine(startPose, shootPose);
        shoot1_collect1 = buildCurve(shootPose, control1, collectPose1);
        collect1_shoot2 = buildLine(collectPose1, shootPose);

        shoot2_collectCurve2 = buildCurve(shootPose, control2, collectCurve2);
        collectCurve2_collectPoint2 = buildLine(collectCurve2, collectPoint2);
        collectPoint2_shoot3 = buildLine(collectPoint2, shootPose);

        shoot3_collectCurve3 = buildCurve(shootPose, control3, collectCurve3);
        collectCurve3_collectPoint3 = buildLine(collectCurve3, collectPoint3);

        collectPoint3_shoot4 = buildLine(collectPoint3, shootPose);
    }

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
    public void loop() {

        follower.update();
        shooter.update();

        switch (state) {

            // ===== ЦИКЛ 1 =====
            case LINE_TO_SHOOT1:
                if (!pathStarted) {
                    follower.followPath(start_shoot1, 0.6, true);
                    pathStarted = true;
                }
                shooter.spinUp();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.SHOOT1;
                }
                break;

            case SHOOT1:
                shooter.spinUp();
                if (shooter.isReady()) shooter.fire();
                if (shooter.isFinished()) {
                    state = State.CURVE_TO_COLLECT1;
                }
                break;

            case CURVE_TO_COLLECT1:
                if (!pathStarted) {
                    follower.followPath(shoot1_collect1, 0.6, true);
                    pathStarted = true;
                }
                intake.start();
                if (!follower.isBusy() && shooter.isFinished()) {
                    timer.resetTimer();
                    state = State.INTAKE1;
                }
                break;

            case INTAKE1:
                intake.start();
                if (timer.getElapsedTimeSeconds() >= 1.0) {
                    pathStarted = false;
                    state = State.LINE_TO_SHOOT2;
                }
                break;

            // ===== ЦИКЛ 2 =====
            case LINE_TO_SHOOT2:
                if (!pathStarted) {
                    follower.followPath(collect1_shoot2, 0.6, true);
                    pathStarted = true;
                }
                shooter.spinUp();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.SHOOT2;
                }
                break;

            case SHOOT2:
                shooter.spinUp();
                if (shooter.isReady()) shooter.fire();
                if (shooter.isFinished()) {
                    state = State.CURVE_TO_COLLECT2;
                }
                break;

            case CURVE_TO_COLLECT2:
                if (!pathStarted) {
                    follower.followPath(shoot2_collectCurve2, 0.6, true);
                    pathStarted = true;
                }
                intake.start();
                if (!follower.isBusy() && shooter.isFinished()) {
                    pathStarted = false;
                    state = State.LINE_COLLECT2;
                }
                break;

            case LINE_COLLECT2:
                if (!pathStarted) {
                    follower.followPath(collectCurve2_collectPoint2, 0.6, true);
                    pathStarted = true;
                }
                intake.start();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.LINE_BACK_TO_SHOOT3;
                }
                break;

            // ===== ЦИКЛ 3 =====
            case LINE_BACK_TO_SHOOT3:
                if (!pathStarted) {
                    follower.followPath(collectPoint2_shoot3, 0.6, true);
                    pathStarted = true;
                }
                shooter.spinUp();
                intake.start();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.SHOOT3;
                }
                break;

            case SHOOT3:
                shooter.spinUp();
                if (shooter.isReady()) shooter.fire();
                if (shooter.isFinished()) {
                    state = State.CURVE_TO_COLLECT3;
                }
                break;

            case CURVE_TO_COLLECT3:
                if (!pathStarted) {
                    follower.followPath(shoot3_collectCurve3, 0.6, true);
                    pathStarted = true;
                }
                intake.start();
                if (!follower.isBusy() && shooter.isFinished()) {
                    pathStarted = false;
                    state = State.LINE_COLLECT3;
                }
                break;

            case LINE_COLLECT3:
                if (!pathStarted) {
                    follower.followPath(collectCurve3_collectPoint3, 0.6, true);
                    pathStarted = true;
                }
                intake.start();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.LINE_TO_SHOOT4;
                }
                break;

            // ===== ФИНАЛЬНЫЙ ВЫСТРЕЛ =====
            case LINE_TO_SHOOT4:
                if (!pathStarted) {
                    follower.followPath(collectPoint3_shoot4, 0.6, true);
                    pathStarted = true;
                }
                shooter.spinUp();
                intake.stop();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    state = State.SHOOT4;
                }
                break;

            case SHOOT4:
                shooter.spinUp();
                if (shooter.isReady()) shooter.fire();
                if (shooter.isFinished()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                shooter.stop();
                intake.stop();
                break;
        }

        telemetry.addData("State", state);
        telemetry.update();
    }
}
