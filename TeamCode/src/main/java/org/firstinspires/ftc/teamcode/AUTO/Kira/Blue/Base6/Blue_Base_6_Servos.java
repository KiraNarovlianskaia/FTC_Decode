package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue.Base6;

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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@Autonomous(name = "Blue_Base_6_Servos", group = "Autonomous")
@Configurable
public class Blue_Base_6_Servos extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Intake intake = new Intake();
    private Shooter shooter = new Shooter();
    private Servos servos = new Servos();
    ElapsedTime timer = new ElapsedTime();
    boolean waitStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23.573, 119.218, Math.toRadians(-90)));

        paths = new Paths(follower);

        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        servos.init(hardwareMap);

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

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.573, 119.218),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(24.560, 118.989),
                                    new Pose(22.901, 89.033)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.901, 89.033),

                                    new Pose(22.901, 80.847)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.901, 80.847),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();
        }
    }

    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                shooter.start();
                follower.followPath(paths.Path1,0.7,true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    servos.servos_shoot();
                    timer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy() && timer.seconds() >= 1) {
                    servos.servos_close();
                    intake.start();
                    follower.followPath(paths.Path2, 0.7, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, 0.7, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, 0.7, true);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    servos.servos_shoot();
                    pathState = 6;
                }
                break;

        }

        return pathState;
    }
}