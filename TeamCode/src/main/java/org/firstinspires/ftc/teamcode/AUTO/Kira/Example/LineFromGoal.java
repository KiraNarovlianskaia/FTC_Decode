package org.firstinspires.ftc.teamcode.AUTO.Kira.Example;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos_Pattern;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@Autonomous(name = "LineFromGoal", group = "Autonomous")
@Configurable
public class LineFromGoal extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    ElapsedTime timer = new ElapsedTime();
    boolean waitStarted = false;

    private final double wheel_speed = 0.5;
    boolean autoFinished = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(12.38, 108.536, Math.toRadians(-90)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        if (autoFinished) return;

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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.38, 108.536),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(24.560, 118.989),
                                    new Pose(23.000, 101.331)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.000, 101.331),

                                    new Pose(23.000, 80.847)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.000, 80.847),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.594, 94.576),

                                    new Pose(23.000, 80.719)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.000, 80.719),

                                    new Pose(23.000, 57.699)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.000, 57.699),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.594, 94.576),

                                    new Pose(22.813, 83.163)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();
        }
    }

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1,wheel_speed,true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, wheel_speed, true);
                    pathState = 2;
                }
                break;

        }

        return pathState;
    }
}