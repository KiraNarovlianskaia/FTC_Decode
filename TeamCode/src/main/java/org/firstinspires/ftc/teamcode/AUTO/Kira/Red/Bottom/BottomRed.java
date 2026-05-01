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
        follower.setStartingPose(new Pose(86.5, 8.799, Math.toRadians(-90)));

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
                                    new Pose(86.5, 8.799),

                                    new Pose(84.7, 12.763)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(250))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.7, 12.763),
                                    new Pose(100, 20.321),
                                    new Pose(124.7, 6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.7, 6),

                                    new Pose(134.5, 6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.5, 6),
                                    new Pose(106.1, 18.924),
                                    new Pose(84.5, 12.583)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.5, 12.583),
                                    new Pose(100, 20.321),
                                    new Pose(125, 6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125, 6),

                                    new Pose(134.5, 6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.5, 6),
                                    new Pose(106, 19.207),
                                    new Pose(85, 12.583)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(240))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85, 12.583),

                                    new Pose(108.2, 12.476)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(270))

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
                if (!follower.isBusy() && actionTimer.seconds() > 2.0) {
                    follower.followPath(paths.Path8, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 8;
                }
                break;
            case 8:
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

    // Класс Paths остается без изменений, как в прошлом сообщении
}