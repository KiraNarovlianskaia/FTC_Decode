package org.firstinspires.ftc.teamcode.AUTO.Kira.Red.Bottom;

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
import org.firstinspires.ftc.teamcode.subsystems.ServosThree;
import org.firstinspires.ftc.teamcode.subsystems.ShooterBottom;

@Autonomous(name = "BottomRed", group = "Autonomous")
@Configurable
public class BottomRed extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private Intake intake = new Intake();
    private ShooterBottom shooter = new ShooterBottom();
    private ServosThree servos = new ServosThree();

    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime actionTimer = new ElapsedTime();

    private final double wheel_speed = 0.5;
    private boolean autoFinished = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(86.526, 8.799, Math.toRadians(270)));

        paths = new Paths(follower);

        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        servos.init(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        if (autoFinished) return;

        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.update(telemetry);
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
            case 0: // Старт
                shooter.start(); // Включаем маховики шутера
                follower.followPath(paths.Path1, wheel_speed, true);
                pathTimer.reset();
                pathState = 1;
                break;

            case 1: // ПОСЛЕ 1-ГО ПРОЕЗДА
                if (pathTimer.seconds() > 3.0) {
                    servos.shootAll();
                    actionTimer.reset();
                    pathState = 11; // Пауза на вылет
                }
                break;

            case 11:
                if (actionTimer.seconds() > 2) {
                    servos.closeAll();
                    intake.start(); // Включаем забор, чтобы собирать по пути
                    follower.followPath(paths.Path2, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) {
                    follower.followPath(paths.Path3, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy() || pathTimer.seconds() > 2.0) {
                    follower.followPath(paths.Path4, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 4;
                }
                break;

            case 4: // ПОСЛЕ 4-ГО ПРОЕЗДА
                if (!follower.isBusy() || pathTimer.seconds() > 3.0) {
                    servos.shootAll();
                    actionTimer.reset();
                    pathState = 41;
                }
                break;

            case 41:
                if (actionTimer.seconds() > 2) {
                    servos.closeAll();
                    follower.followPath(paths.Path5, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) {
                    follower.followPath(paths.Path6, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (!follower.isBusy() || pathTimer.seconds() > 2.0) {
                    follower.followPath(paths.Path7, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 7;
                }
                break;

            case 7: // ПОСЛЕ 7-ГО ПРОЕЗДА
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) {
                    servos.shootAll();
                    actionTimer.reset();
                    pathState = 71;
                }
                break;

            case 71:
                if (actionTimer.seconds() > 2) {
                    servos.closeAll();
                    intake.stop();
                    shooter.stop();
                    autoFinished = true;
                }
                break;
        }
        return pathState;
    }

    // Класс Paths остается без изменений, как в прошлом сообщении
}