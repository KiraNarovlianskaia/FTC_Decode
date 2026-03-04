package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue.Base6;

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

@Autonomous(name = "Blue_Base_6_Camera", group = "Autonomous")
@Configurable
public class Blue_Base_9_Camera extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Intake intake = new Intake();
    private Shooter shooter = new Shooter();
    private Servos_Pattern servos = new Servos_Pattern();
    ElapsedTime timer = new ElapsedTime();
    boolean waitStarted = false;

    private Limelight3A limelight;
    private int detectedTagId = -1;      // последнее увиденное
    private int finalTagId = 21;         // зафиксированное перед стартом
    private int shoot_id = 1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23.573, 119.218, Math.toRadians(-90)));

        paths = new Paths(follower);

        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        servos.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);   // pipeline AprilTag
        limelight.setPollRateHz(30);
        limelight.start();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void init_loop() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (!fiducials.isEmpty()) {
                detectedTagId = fiducials.get(0).getFiducialId();
            }
        }

        panelsTelemetry.debug("Last Seen Tag", detectedTagId);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        finalTagId = detectedTagId;  // фиксируем последний увиденный
    }

    @Override
    public void loop() {
        follower.update();
        servos.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    private int getVariant(int shootId) {
        switch (finalTagId) {
            case 21:
                if (shootId == 1) return 1;
                if (shootId == 3) return 2;
                if (shootId == 4) return 3;
                if (shootId == 2) return 4;
                break;
            case 22:
                if (shootId == 1 || shootId == 2) return 2;
                if (shootId == 3) return 1;
                if (shootId == 4) return 4;
                break;
            case 23:
                if (shootId == 1 || shootId == 2) return 3;
                if (shootId == 3) return 4;
                if (shootId == 4) return 1;
                break;
        }
        // по умолчанию, если не попало под первый вариант
        return 1;
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
                                    new Pose(22.902, 89.033)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.902, 89.033),

                                    new Pose(22.902, 80.847)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.902, 80.847),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(25.869, 84.504),
                                    new Pose(24.894, 74.719)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.894, 74.719),

                                    new Pose(24.894, 60.848)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.894, 60.848),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(30.302, 78.164),
                                    new Pose(24.045, 53.170)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.045, 53.170),

                                    new Pose(24.045, 36.079)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.045, 36.079),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.594, 94.576),

                                    new Pose(48.594, 69.648)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))

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
                    int shootId = 1; // пример, можешь выбрать динамически
                    int variant = getVariant(shootId); // получаем вариант
                    servos.startShooting(shootId, variant);
                    timer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy() && timer.seconds() >= 2.1) {
                    servos.closeAll();
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
                    int shootId = 2; // пример, можешь выбрать динамически
                    int variant = getVariant(shootId); // получаем вариант
                    servos.startShooting(shootId, variant);
                    pathState = 6;
                }
                break;

        }

        return pathState;
    }
}