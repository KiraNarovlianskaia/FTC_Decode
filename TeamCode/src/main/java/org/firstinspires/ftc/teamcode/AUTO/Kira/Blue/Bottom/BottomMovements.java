package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue.Bottom;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BottomMovements", group = "Autonomous")
@Configurable
public class BottomMovements extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // Таймер для предотвращения застреваний
    private ElapsedTime pathTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57.474, 8.799, Math.toRadians(-90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Timer", pathTimer.seconds());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }



    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.474, 8.799),

                                    new Pose(59.318, 12.763)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-60))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.318, 12.763),
                                    new Pose(44.006, 20.321),
                                    new Pose(19.247, 9.557)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.247, 9.557),

                                    new Pose(9.477, 9.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.477, 9.517),
                                    new Pose(37.917, 18.924),
                                    new Pose(59.495, 12.583)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-60))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.495, 12.583),
                                    new Pose(44.006, 20.321),
                                    new Pose(19.247, 9.557)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.247, 9.557),

                                    new Pose(9.477, 9.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.477, 9.517),
                                    new Pose(38.073, 19.207),
                                    new Pose(59.209, 12.583)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-60))

                    .build();
        }
    }



    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Запускаем первый путь и сбрасываем таймер
                follower.followPath(paths.Path1, 0.5, true);
                pathTimer.reset();
                pathState = 1;
                break;

            case 1:
                // Если робот доехал ИЛИ прошло больше 3 секунд
                if (!follower.isBusy() || pathTimer.seconds() > 3.0) {
                    follower.followPath(paths.Path2, 0.5, true);
                    pathTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) { // Для длинной кривой увеличил до 4с
                    follower.followPath(paths.Path3, 0.5, true);
                    pathTimer.reset();
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy() || pathTimer.seconds() > 2.0) {
                    follower.followPath(paths.Path4, 0.5, true);
                    pathTimer.reset();
                    pathState = 4;
                }
                break;
            case 4:
                // Если робот доехал ИЛИ прошло больше 3 секунд
                if (!follower.isBusy() || pathTimer.seconds() > 3.0) {
                    follower.followPath(paths.Path5, 0.5, true);
                    pathTimer.reset();
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) { // Для длинной кривой увеличил до 4с
                    follower.followPath(paths.Path6, 0.5, true);
                    pathTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy() || pathTimer.seconds() > 2.0) {
                    follower.followPath(paths.Path7, 0.5, true);
                    pathTimer.reset();
                    pathState = 7;
                }
                break;


            case 7:
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) {
                    // Путь завершен, можно добавить состояние ожидания или стоп
                    pathState = 8;
                }
                break;
        }

        return pathState;
    }
}