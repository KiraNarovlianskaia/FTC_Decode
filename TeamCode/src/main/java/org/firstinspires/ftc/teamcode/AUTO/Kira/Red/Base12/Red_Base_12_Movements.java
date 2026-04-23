package org.firstinspires.ftc.teamcode.AUTO.Kira.Red.Base12;

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

@Autonomous(name = "Red_Base_12_Movements", group = "Autonomous")
@Configurable
public class Red_Base_12_Movements extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime pathTimer = new ElapsedTime();
    private final double wheel_speed = 1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Отражаем X: 144 - 30.822 = 113.178
        // Отражаем Угол: 180 - 180 = 0
        follower.setStartingPose(new Pose(113.178, 129.292, Math.toRadians(0)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized Red (X-Reflected)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(113.178, 129.292),
                                    new Pose(95.406, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))
                    .build();

            // Path 2
            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.406, 94.576),
                                    new Pose(108.275, 113.550),
                                    new Pose(119.855, 101.331)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(270)) // 180 - (-90) = 270
                    .build();

            // Path 3
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.855, 101.331),
                                    new Pose(119.855, 80.847)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            // Path 4
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.855, 80.847),
                                    new Pose(130.008, 75.328)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            // Path 5
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.008, 75.328),
                                    new Pose(95.406, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-135))
                    .build();

            // Path 6
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.406, 94.576),
                                    new Pose(119.569, 79.574)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(270))
                    .build();

            // Path 7
            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.569, 79.574),
                                    new Pose(120.141, 57.985)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            // Path 8
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.141, 57.985),
                                    new Pose(95.406, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-135))
                    .build();

            // Path 9
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.406, 94.576),
                                    new Pose(120.328, 50.386)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(270))
                    .build();

            // Path 10
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.328, 50.386),
                                    new Pose(120.328, 29.328)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            // Path 11
            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.328, 29.328),
                                    new Pose(95.467, 94.628)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-135))
                    .build();

            // Path 12
            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.467, 94.628),
                                    new Pose(119.439, 79.455)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(270))
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
                    pathTimer.reset();
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