package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue.Base9;

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
import org.firstinspires.ftc.teamcode.subsystems.Servos;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Blue_Base_9_SENSORS", group = "Autonomous")
@Configurable
public class Blue_Base_9_Sensors extends OpMode {

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

    private final double wheel_speed = 0.5;
    boolean autoFinished = false;

    // 🔥 ПАТТЕРН
    private List<Servos.BallColor> pattern = new ArrayList<>();

    // ================= INIT =================
    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(27.996, 119.643, Math.toRadians(180)));

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
        servos.addTelemetryData(gamepad1, telemetry);

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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(37.996, 119.643),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(24.560, 118.989),
                                    new Pose(24.145, 101.331)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.145, 101.331),

                                    new Pose(23.573, 80.847)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.573, 80.847),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.594, 94.576),

                                    new Pose(24.431, 79.574)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.431, 79.574),

                                    new Pose(23.859, 57.985)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.859, 57.985),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.594, 94.576),

                                    new Pose(23.672, 80.873)
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
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    servos.startShooting(pattern);
                    timer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (timer.seconds() >= 3) {
                    servos.closeAll();
                    follower.followPath(paths.Path5, wheel_speed, true);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
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
                    servos.startShooting(pattern);
                    timer.reset();
                    pathState = 10;
                }
                break;

            case 10:
                if (timer.seconds() >= 3) {
                    servos.closeAll();
                    follower.followPath(paths.Path8, wheel_speed, true);
                    pathState = 11;
                }
                break;

            case 11:
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