package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoRed_9", group = "Tests")
public class AutoRed_9 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState {
        CURVE_TO_COLLECT,
        LINE_TO_POINT1,
        LINE_TO_POINT2,
        LINE_TO_POINT3,
        LINE_TO_POINT4,
        CURVE_TO_SHOOT,
        DONE
    }
    PathState pathState;

    private final Pose startPose = new Pose(119.84, 118.79, Math.toRadians(0));
    private final Pose collectPose = new Pose(101.232, 84.38, Math.toRadians(0));

    private final Pose point1 = new Pose(119.69, 84.04, Math.toRadians(0));
    private final Pose point2 = new Pose(86.26, 84.63, Math.toRadians(45));
    private final Pose point3 = new Pose(101.38, 60.23, Math.toRadians(0));
    private final Pose point4 = new Pose(120.03, 59.56, Math.toRadians(0));
    private final Pose control1 = new Pose(92.18, 108.78, 0);
    private final Pose control2 = new Pose(84.22, 66.85, 0);

    private PathChain curvePath1;
    private PathChain linePath1;
    private PathChain linePath2;
    private PathChain curvePath2;
    private PathChain linePath3;
    private PathChain linePath4;

    private void followAndAdvance(PathChain path, PathState next) {
        if (!follower.isBusy()) {
            follower.followPath(path, 0.5, true);
            pathState = next;
        }
    }

    private PathChain buildLine(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(
                        from.getHeading(),
                        to.getHeading()
                )
                .build();
    }

    private PathChain buildCurve(Pose from, Pose control, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(from, control, to))
                .setLinearHeadingInterpolation(
                        from.getHeading(),
                        to.getHeading()
                )
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();

        pathState = PathState.CURVE_TO_COLLECT;
        follower.setPose(startPose);
        buildPaths();
    }

    private void buildPaths() {
        curvePath1 = buildCurve(startPose,control1,collectPose);
        linePath1 = buildLine(collectPose,point1);
        linePath2 = buildLine(point1,point2);
        curvePath2 = buildCurve(point2,control2,point3);
        linePath3 = buildLine(point3,point4);
        linePath4 = buildLine(point4,point2);
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

            case CURVE_TO_COLLECT:
                follower.followPath(curvePath1, 0.5, true);
                pathState = PathState.LINE_TO_POINT1;
                break;

            case LINE_TO_POINT1:
                followAndAdvance(linePath1, PathState.LINE_TO_POINT2);
                break;

            case LINE_TO_POINT2:
                followAndAdvance(linePath2, PathState.CURVE_TO_SHOOT);
                break;

            case CURVE_TO_SHOOT:
                followAndAdvance(curvePath2, PathState.LINE_TO_POINT3);
                break;

            case LINE_TO_POINT3:
                followAndAdvance(linePath3, PathState.LINE_TO_POINT4);
                break;

            case LINE_TO_POINT4:
                followAndAdvance(linePath4, PathState.DONE);
                break;

            case DONE:
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.update();
    }

}