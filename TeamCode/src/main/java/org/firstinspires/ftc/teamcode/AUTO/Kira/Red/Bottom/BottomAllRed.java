package org.firstinspires.ftc.teamcode.AUTO.Kira.Red.Bottom; // Предположим, теперь это Blue

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
import org.firstinspires.ftc.teamcode.subsystems.Servos_Pattern_Blue;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "BottomAll Red", group = "Autonomous")
@Configurable
public class BottomAllRed extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Intake intake = new Intake();
    private Shooter shooter = new Shooter();
    private Servos_Pattern_Blue servos = new Servos_Pattern_Blue();
    private ElapsedTime pathTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    boolean waitStarted = false;

    private int detectedTagId = -1;
    private int finalTagId = -1;
    private int shoot_id = 1;

    private final double wheel_speed = 0.5;
    boolean autoFinished = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Отражение по X: 144 - 57.474 = 86.526
        // Отражение угла: 180 - (-90) = 270 (или -90)
        follower.setStartingPose(new Pose(86.526, 8.799, Math.toRadians(270)));

        paths = new Paths(follower);

        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        servos.init(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized (X-Reflected)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        finalTagId = detectedTagId;
    }

    @Override
    public void loop() {
        if (autoFinished) return;

        follower.update();
        servos.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    private int getVariant(int shootId) {
        // Здесь логика тегов может потребовать изменений в зависимости от ID на другой стороне
        switch (finalTagId) {
            case 21:
                if (shootId == 1 || shootId == 2) return 1;
                if (shootId == 3) return 2;
                if (shootId == 4) return 3;
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
            case -1:
                return 1;
        }
        return 1;
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

        public Paths(Follower follower) {
            // Path 1
            // 144 - 57.474 = 86.526 | 144 - 59.318 = 84.682
            // Angles: 180 - (-90) = 270 | 180 - (-60) = 240
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.526, 8.799),
                                    new Pose(84.682, 12.763)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(240))
                    .build();

            // Path 2
            // 144 - 59.318 = 84.682 | 144 - 44.006 = 99.994 | 144 - 19.247 = 124.753
            // Angles: 180 - (-60) = 240 | 180 - (-180) = 360 (0)
            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.682, 12.763),
                                    new Pose(99.994, 20.321),
                                    new Pose(124.753, 9.557)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(360))
                    .build();

            // Path 3
            // 144 - 19.247 = 124.753 | 144 - 9.477 = 134.523
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.753, 9.557),
                                    new Pose(134.523, 9.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(360))
                    .build();

            // Path 4
            // Reverse curve
            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.523, 9.517),
                                    new Pose(106.083, 18.924),
                                    new Pose(84.505, 12.583)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(240))
                    .build();

            // Path 5
            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.505, 12.583),
                                    new Pose(99.994, 20.321),
                                    new Pose(124.753, 9.557)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(360))
                    .build();

            // Path 6
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.753, 9.557),
                                    new Pose(134.523, 9.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(360))
                    .build();

            // Path 7
            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.523, 9.517),
                                    new Pose(105.927, 19.207),
                                    new Pose(84.791, 12.583)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(240))
                    .build();

            // Path 8
            // 144 - 59.209 = 84.791 | 144 - 59.449 = 84.551
            // Angle: 180 - (-60) = 240 | 180 - (-90) = 270
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.791, 12.583),
                                    new Pose(84.551, 33.213)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(270))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.start();
                follower.followPath(paths.Path1, wheel_speed, true);
                pathTimer.reset();
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    int shootId = 1;
                    int variant = getVariant(shootId);
                    servos.startShooting(shootId, variant);
                    timer.reset();
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy() && timer.seconds() >= 3) {
                    servos.closeAll();
                    intake.start();
                    follower.followPath(paths.Path2, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) {
                    follower.followPath(paths.Path3, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy() || pathTimer.seconds() > 2.0) {
                    follower.followPath(paths.Path4, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    int shootId = 2;
                    int variant = getVariant(shootId);
                    timer.reset();
                    pathTimer.reset();
                    servos.startShooting(shootId, variant);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy() && timer.seconds() >= 3) {
                    servos.closeAll();
                    follower.followPath(paths.Path5, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) {
                    follower.followPath(paths.Path6, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy() || pathTimer.seconds() > 2.0) {
                    follower.followPath(paths.Path7, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    int shootId = 3;
                    int variant = getVariant(shootId);
                    timer.reset();
                    pathTimer.reset();
                    servos.startShooting(shootId, variant);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy() && timer.seconds() >= 3) {
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