package org.firstinspires.ftc.teamcode.AUTO.Kira.Example;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name = "Curve", group = "Tests")
public class Curve extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS,
        DONE
    }

    private PathState pathState;

    private final Pose startPose = new Pose(38, 33, Math.toRadians(90));
    private final Pose shootPose = new Pose(24.62027833001988, 118.23459244532802, Math.toRadians(-90));

    // контрольные точки как обычные Pose
    private final Pose control1 = new Pose(83.39363385685887, 75.85685906560637, 0);
    //private final Pose control2 = new Pose(9.733598, 38.075547, 0);

    private PathChain driveStartPosShootPos;
    //private PathChain driveShootPosEndPos; // конечный путь — задай сам

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;

        // установка стартовой позиции локации
        follower.setPose(startPose);

        buildPaths();
    }

    private void buildPaths() {
        // Bézier-кривая от startPose через две контрольные точки до shootPose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, control1, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // пример второго пути — прямой путь от shootPose к другой позиции
        // задай правильные финальные координаты

    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        // обязательно обновляем follower каждый кадр
        follower.update();

        // FSM для перехода по путям
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, 0.5,true);  // запускаем первый путь
                pathState = PathState.SHOOT_PRELOAD;
                pathTimer.resetTimer();
                break;

            case SHOOT_PRELOAD:
                // check is follower done it's path?
                if (!follower.isBusy()){
                    // TODO add logic to shooter
                    telemetry.addLine("Done Path l");
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                if (!follower.isBusy()) {
                    pathState = PathState.DONE;
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }

        telemetry.addData("State", pathState.toString());
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
