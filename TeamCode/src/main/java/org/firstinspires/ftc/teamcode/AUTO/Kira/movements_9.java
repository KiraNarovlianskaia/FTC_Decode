package org.firstinspires.ftc.teamcode.AUTO.Kira;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Movements9", group = "Paths")
public class movements_9 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState {
        LINE_TO_SHOOT,
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
    PathState pathState;

    // Позиции
    private final Pose startPose = new Pose(24.508, 119.077, Math.toRadians(-90));
    private final Pose collectPose1 = new Pose(24.333, 89.033, Math.toRadians(-90));
    private final Pose shootPose = new Pose(48.594, 94.576, Math.toRadians(135));
    private final Pose collectCurve2 = new Pose(24.894, 74.719, Math.toRadians(-90));
    private final Pose collectPoint2 = new Pose(24.894, 65.848, Math.toRadians(-90));
    private final Pose collectCurve3 = new Pose(24.045, 53.170, Math.toRadians(-90));
    private final Pose collectPoint3 = new Pose(24.045, 40.079, Math.toRadians(-90));
    private final Pose control1 = new Pose(24.56, 118.989, 0);
    private final Pose control2 = new Pose(25.869, 84.504, 0);
    private final Pose control3 = new Pose(30.302, 78.164, 0);

    // Пути
    private PathChain startPose_shootPose;
    private PathChain shootPose_collectPose1;
    private PathChain collectPose1_shootPose;
    private PathChain shootPose_collectCurve2;
    private PathChain collectCurve2_collectPoint2;
    private PathChain collectPoint2_shootPose;
    private PathChain shootPose_collectCurve3;
    private PathChain collectCurve3_collectPoint3;
    private PathChain collectPoint3_shootPose;

    private void followAndAdvance(PathChain path, PathState next) {
        if (!follower.isBusy()) {
            follower.followPath(path, 0.5, true);
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

        pathTimer = new Timer();
        opModeTimer = new Timer();

        pathState = PathState.CURVE_TO_COLLECT1;
        follower.setPose(startPose);
        buildPaths();
    }

    private void buildPaths() {
        startPose_shootPose = buildLine(startPose, shootPose);
        shootPose_collectPose1 = buildCurve(shootPose, control1, collectPose1);
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

        switch (pathState) {
            case LINE_TO_SHOOT:
                followAndAdvance(startPose_shootPose, PathState.CURVE_TO_COLLECT1);
                break;

            case CURVE_TO_COLLECT1:
                followAndAdvance(shootPose_collectPose1, PathState.LINE_TO_SHOOT1);
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
                // Можно включить тормоза или остановить follower
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.update();
    }
}
