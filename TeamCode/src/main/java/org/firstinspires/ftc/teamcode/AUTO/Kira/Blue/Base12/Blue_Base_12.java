package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue.Base12;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;
@Autonomous(name = "Blue_Base_12", group = "Autonomous")
@Configurable
public class Blue_Base_12 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private Intake intake = new Intake();

    private Shooter shooter = new Shooter();
    private Servos servos = new Servos();

    ElapsedTime timer = new ElapsedTime();

    private Limelight3A limelight;
    private int detectedTagId = -1;
    private int finalTagId = -1;

    private final double wheel_speed = 0.7;
    boolean autoFinished = false;

    // 🔥 ПАТТЕРН
    private List<Servos.BallColor> pattern = new ArrayList<>();
    private ElapsedTime pathTimer = new ElapsedTime(); // Добавлено

    // ================= INIT =================
    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(30.822, 129.292, Math.toRadians(180)));

        paths = new Paths(follower);

        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        servos.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.setPollRateHz(30);
        limelight.start();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // ================= INIT LOOP =================
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

    // ================= START =================
    @Override
    public void start() {

        finalTagId = detectedTagId;

        // 🔥 формируем паттерн
        pattern = getPatternFromTag(finalTagId);
    }

    // ================= LOOP =================
    @Override
    public void loop() {
        if (autoFinished) return;

        follower.update();
        servos.update();

        pathState = autonomousPathUpdate();

        // --- Твоя старая телеметрия ---
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("Tag", finalTagId);

        // --- НОВАЯ ТЕЛЕМЕТРИЯ ДЛЯ ДАТЧИКОВ ---
        servos.addTelemetryData(gamepad1,telemetry);

        panelsTelemetry.update(telemetry);
    }

    // ================= ПАТТЕРН =================
    private List<Servos.BallColor> getPatternFromTag(int tag) {

        List<Servos.BallColor> p = new ArrayList<>();

        switch (tag) {

            case 21:
                p.add(Servos.BallColor.GREEN);
                p.add(Servos.BallColor.PURPLE);
                p.add(Servos.BallColor.PURPLE);
                break;

            case 22:
                p.add(Servos.BallColor.PURPLE);
                p.add(Servos.BallColor.GREEN);
                p.add(Servos.BallColor.PURPLE);
                break;

            case 23:
                p.add(Servos.BallColor.PURPLE);
                p.add(Servos.BallColor.PURPLE);
                p.add(Servos.BallColor.GREEN);
                break;

            default:
                p.add(Servos.BallColor.GREEN);
                p.add(Servos.BallColor.PURPLE);
                p.add(Servos.BallColor.PURPLE);
                break;
        }

        return p;
    }

    // ================= PATHS (БЕЗ ИЗМЕНЕНИЙ) =================

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

    // ================= АВТО ЛОГИКА =================
    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                shooter.start();
                follower.followPath(paths.Path1, wheel_speed, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    servos.startShooting(pattern);
                    timer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (timer.seconds() >= 3) {
                    servos.closeAll();
                    intake.start();
                    follower.followPath(paths.Path2, wheel_speed, true);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, wheel_speed, true);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy() || pathTimer.seconds() > 2) {
                    follower.followPath(paths.Path5, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    servos.startShooting(pattern);
                    timer.reset();
                    pathState = 7;
                }
                break;

            case 7:
                if (timer.seconds() >= 3) {
                    servos.closeAll();
                    follower.followPath(paths.Path6, wheel_speed, true);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, wheel_speed, true);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, wheel_speed, true);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    servos.startShooting(pattern);
                    timer.reset();
                    pathState = 11;
                }
                break;

            case 11:
                if (timer.seconds() >= 3) {
                    servos.closeAll();
                    follower.followPath(paths.Path9, wheel_speed, true);
                    pathState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, wheel_speed, true);
                    pathState = 13;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11, wheel_speed, true);
                    pathState = 14;
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    servos.startShooting(pattern);
                    timer.reset();
                    pathState = 15;
                }
                break;
            case 15:
                if (timer.seconds() >= 3) {
                    servos.closeAll();
                    follower.followPath(paths.Path12, wheel_speed, true);
                    pathState = 16;
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    intake.stop();
                    shooter.stop();
                    servos.closeAll();

                    autoFinished = true;
                }
                break;
        }

        return pathState;
    }
}