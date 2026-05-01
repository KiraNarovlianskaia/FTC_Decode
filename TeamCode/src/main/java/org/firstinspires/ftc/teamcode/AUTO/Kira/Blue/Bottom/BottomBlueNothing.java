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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ServosThree;
import org.firstinspires.ftc.teamcode.subsystems.ShooterBottom;

@Autonomous(name = "BottomBlueNothing", group = "Autonomous")
@Configurable
public class BottomBlueNothing extends OpMode {

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
        follower.setStartingPose(new Pose(57.474, 8.799, Math.toRadians(-90)));

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
                                    new Pose(57.474, 8.799),

                                    new Pose(59.318, 12.763)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-70))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.209, 12.583),

                                    new Pose(59.449, 33.213)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-70), Math.toRadians(-90))

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
                    //intake.start(); // Включаем забор, чтобы собирать по пути
                    follower.followPath(paths.Path2, wheel_speed, true);
                    pathTimer.reset();
                    pathState = 2;
                }
                break;



            case 2:
                if (!follower.isBusy()) {
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