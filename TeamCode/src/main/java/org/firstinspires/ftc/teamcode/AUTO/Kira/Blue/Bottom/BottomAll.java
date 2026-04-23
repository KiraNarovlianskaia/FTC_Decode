package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue.Bottom;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Servos_Pattern;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;


@Autonomous(name = "BottomAll", group = "Autonomous")
@Configurable
public class BottomAll extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Intake intake = new Intake();
    private Shooter shooter = new Shooter();
    private Servos_Pattern servos = new Servos_Pattern();
    private ElapsedTime pathTimer = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    boolean waitStarted = false;


    private int detectedTagId = -1;      // последнее увиденное
    private int finalTagId = -1;         // зафиксированное перед стартом
    private int shoot_id = 1;

    private final double wheel_speed = 0.5;
    boolean autoFinished = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(57.474, 8.799, Math.toRadians(-90)));
        paths = new Paths(follower);

        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        servos.init(hardwareMap);



        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        finalTagId = detectedTagId;  // фиксируем последний увиденный
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
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    private int getVariant(int shootId) {
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
                                    new Pose(57.474, 8.799),

                                    new Pose(59.318, 12.763)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-60))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.318, 12.763),
                                    new Pose(44.006, 20.321),
                                    new Pose(19.247, 9.557)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.247, 9.557),

                                    new Pose(9.477, 9.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.477, 9.517),
                                    new Pose(37.917, 18.924),
                                    new Pose(59.495, 12.583)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-60))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.495, 12.583),
                                    new Pose(44.006, 20.321),
                                    new Pose(19.247, 9.557)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.247, 9.557),

                                    new Pose(9.477, 9.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.477, 9.517),
                                    new Pose(38.073, 19.207),
                                    new Pose(59.209, 12.583)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-60))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.209, 12.583),

                                    new Pose(59.449, 33.213)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-90))

                    .build();
        }
    }


    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                shooter.start();
                follower.followPath(paths.Path1,wheel_speed,true);
                pathTimer.reset();
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
                if (!follower.isBusy() && timer.seconds() >= 3) {
                    servos.closeAll();
                    intake.start();
                    follower.followPath(paths.Path2, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) { // Для длинной кривой увеличил до 4с
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
                    int shootId = 2; // пример, можешь выбрать динамически
                    int variant = getVariant(shootId); // получаем вариант
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
                if (!follower.isBusy() || pathTimer.seconds() > 4.0) { // Для длинной кривой увеличил до 4с
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
                    int shootId = 3; // пример, можешь выбрать динамически
                    int variant = getVariant(shootId); // получаем вариант
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