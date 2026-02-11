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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "AutoBlue_9_Ext", group = "Paths")
public class allin_9 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private Intake intake;
    private Shooter shooter;

    private enum PathState {
        CURVE_TO_COLLECT1,
        LINE_TO_SHOOT1,
        CURVE_TO_COLLECT2,
        LINE_TO_COLLECT2,
        LINE_BACK_TO_SHOOT2,
        CURVE_TO_COLLECT3,
        LINE_TO_COLLECT3,
        LINE_BACK_TO_SHOOT3,
        DONE
    }
    private PathState pathState;

    // Позиции
    private final Pose startPose = new Pose(24.508, 119.077, Math.toRadians(-90));
    private final Pose collectPose1 = new Pose(24.508, 90, Math.toRadians(-90));
    private final Pose shootPose = new Pose(48.594, 94.576, Math.toRadians(-45));
    private final Pose collectCurve2 = new Pose(24.894, 74.719, Math.toRadians(-90));
    private final Pose collectPoint2 = new Pose(24.894, 65.848, Math.toRadians(-90));
    private final Pose collectCurve3 = new Pose(24.045, 53.170, Math.toRadians(-90));
    private final Pose collectPoint3 = new Pose(24.045, 40.079, Math.toRadians(-90));
    private final Pose control2 = new Pose(25.869, 84.504, 0);
    private final Pose control3 = new Pose(30.302, 78.164, 0);

    // Пути
    private PathChain startPose_collectPose1;
    private PathChain collectPose1_shootPose;
    private PathChain shootPose_collectCurve2;
    private PathChain collectCurve2_collectPoint2;
    private PathChain collectPoint2_shootPose;
    private PathChain shootPose_collectCurve3;
    private PathChain collectCurve3_collectPoint3;
    private PathChain collectPoint3_shootPose;

    private boolean pathStarted = false;

    private void followAndAdvance(PathChain path, PathState next) {
        if (!follower.isBusy() && !pathStarted) {
            follower.followPath(path, 0.5, true);
            pathStarted = true;
            pathState = next;
        }
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
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        intake = new Intake(hardwareMap.get(
                com.qualcomm.robotcore.hardware.DcMotor.class, Constants.intake));

        shooter = new Shooter(
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, Constants.shooter),
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, Constants.servoL),
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, Constants.servoR)
        );

        pathTimer = new Timer();
        opModeTimer = new Timer();

        pathState = PathState.CURVE_TO_COLLECT1;
        follower.setPose(startPose);
        buildPaths();
    }

    private void buildPaths() {
        startPose_collectPose1 = buildLine(startPose, collectPose1);
        collectPose1_shootPose = buildLine(collectPose1, shootPose);

        shootPose_collectCurve2 = buildCurve(shootPose, control2, collectCurve2);
        collectCurve2_collectPoint2 = buildLine(collectCurve2, collectPoint2);
        collectPoint2_shootPose = buildLine(collectPoint2, shootPose);

        shootPose_collectCurve3 = buildCurve(shootPose, control3, collectCurve3);
        collectCurve3_collectPoint3 = buildLine(collectCurve3, collectPoint3);
        collectPoint3_shootPose = buildLine(collectPoint3, shootPose);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        intake.update();
        shooter.update();

        // Включаем Intake и Shooter на нужных проходах
        switch (pathState) {
            case CURVE_TO_COLLECT1:
            case LINE_TO_COLLECT2:
            case LINE_TO_COLLECT3:
                intake.start(); // Intake на 1, 4, 7
                break;

            case LINE_TO_SHOOT1:
            case LINE_BACK_TO_SHOOT2:
            case LINE_BACK_TO_SHOOT3:
                shooter.spinUp(); // Раскрутка на 2, 5, 8
                break;

            default:
                break;
        }

        // Делаем выстрел только когда на shootPose
        if ((pathState == PathState.LINE_TO_SHOOT1 && !follower.isBusy()) ||
                (pathState == PathState.LINE_BACK_TO_SHOOT2 && !follower.isBusy()) ||
                (pathState == PathState.LINE_BACK_TO_SHOOT3 && !follower.isBusy())) {
            if (shooter.isReady()) {
                shooter.fire();
            }
        }

        // Обновление движения и переход к следующему пути
        switch (pathState) {

            case CURVE_TO_COLLECT1:
                followAndAdvance(startPose_collectPose1, PathState.LINE_TO_SHOOT1);
                break;

            case LINE_TO_SHOOT1:
                followAndAdvance(collectPose1_shootPose, PathState.CURVE_TO_COLLECT2);
                break;

            case CURVE_TO_COLLECT2:
                followAndAdvance(shootPose_collectCurve2, PathState.LINE_TO_COLLECT2);
                break;

            case LINE_TO_COLLECT2:
                followAndAdvance(collectCurve2_collectPoint2, PathState.LINE_BACK_TO_SHOOT2);
                break;

            case LINE_BACK_TO_SHOOT2:
                followAndAdvance(collectPoint2_shootPose, PathState.CURVE_TO_COLLECT3);
                break;

            case CURVE_TO_COLLECT3:
                followAndAdvance(shootPose_collectCurve3, PathState.LINE_TO_COLLECT3);
                break;

            case LINE_TO_COLLECT3:
                followAndAdvance(collectCurve3_collectPoint3, PathState.LINE_BACK_TO_SHOOT3);
                break;

            case LINE_BACK_TO_SHOOT3:
                followAndAdvance(collectPoint3_shootPose, PathState.DONE);
                break;

            case DONE:
                intake.stop();
                shooter.stop();
                break;
        }

        // Сброс флага для следующего пути
        if (!follower.isBusy()) {
            pathStarted = false;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("Shooter Busy", shooter.isBusy());
        telemetry.addData("Intake Busy", intake.isBusy());
        telemetry.update();
    }
}
