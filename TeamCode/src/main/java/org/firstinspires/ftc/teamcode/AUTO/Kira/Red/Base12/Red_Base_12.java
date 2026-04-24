package org.firstinspires.ftc.teamcode.AUTO.Kira.Red.Base12;

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
@Autonomous(name = "Red_Base_12", group = "Autonomous")
@Configurable
public class Red_Base_12 extends OpMode {

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