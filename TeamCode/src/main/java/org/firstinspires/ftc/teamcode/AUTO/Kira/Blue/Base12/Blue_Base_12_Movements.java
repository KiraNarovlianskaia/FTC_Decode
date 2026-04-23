package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue.Base12;

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
import com.qualcomm.robotcore.util.ElapsedTime; // Добавлено

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue_Base_12_Movements", group = "Autonomous")
@Configurable
public class Blue_Base_12_Movements extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime pathTimer = new ElapsedTime(); // Добавлено
    private final double wheel_speed = 1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(30.822, 129.292, Math.toRadians(180)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
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
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30.822, 129.292),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(35.725, 113.550),
                                    new Pose(24.145, 101.331)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.145, 101.331),

                                    new Pose(24.145, 80.847)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.145, 80.847),

                                    new Pose(13.992, 75.328)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.992, 75.328),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.594, 94.576),

                                    new Pose(24.431, 79.574)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.431, 79.574),

                                    new Pose(23.859, 57.985)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.859, 57.985),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.594, 94.576),

                                    new Pose(23.672, 50.386)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.672, 50.386),

                                    new Pose(23.672, 29.328)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.672, 29.328),

                                    new Pose(48.533, 94.628)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.533, 94.628),

                                    new Pose(24.561, 79.455)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, wheel_speed, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, wheel_speed, true);
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, wheel_speed, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, wheel_speed, true);
                    pathTimer.reset(); // Сбрасываем таймер перед началом Path4
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy() || pathTimer.seconds() > 2) {
                    follower.followPath(paths.Path5, wheel_speed, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, wheel_speed, true);
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, wheel_speed, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, wheel_speed, true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, wheel_speed, true);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, wheel_speed, true);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, wheel_speed, true);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path12, wheel_speed, true);
                    pathState = 12;
                }
                break;
        }

        return pathState;
    }
}