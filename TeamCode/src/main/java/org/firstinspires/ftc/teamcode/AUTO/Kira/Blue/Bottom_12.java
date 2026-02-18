
package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Autonomous(name = "Blue_Bottom_12", group = "Autonomous")
@Configurable // Panels
public class Bottom_12 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Shooter shooter = new Shooter();
    private boolean shotsTriggered = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        shooter.init(hardwareMap);
        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        shooter.update();
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
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
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.859, 9.145),

                                    new Pose(56.859, 14.871)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.859, 14.871),

                                    new Pose(24.286, 23.101)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.286, 23.101),

                                    new Pose(24.286, 30.225)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.286, 30.225),

                                    new Pose(56.859, 14.871)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.859, 14.871),

                                    new Pose(24.481, 46.127)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.481, 46.127),

                                    new Pose(24.481, 55.040)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.481, 55.040),

                                    new Pose(56.859, 14.871)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.859, 14.871),

                                    new Pose(23.817, 70.070)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.817, 70.070),

                                    new Pose(23.817, 80.650)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.817, 80.650),

                                    new Pose(56.859, 14.871)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))

                    .build();
        }
    }




    public int autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1: // Shoot after Path1
                if (!follower.isBusy()) {

                    if (!shotsTriggered) {
                        shooter.shoot();
                        shotsTriggered = true;
                    }

                    if (shotsTriggered && !shooter.isBusy()) {
                        shotsTriggered = false;
                        pathState = 2;
                    }
                }
                break;

            case 2:
                follower.followPath(paths.Path2);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 4;
                }
                break;

            case 4: // Shoot after Path3
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pathState = 5;
                }
                break;


            case 5:
                if (!follower.isBusy()) {

                    if (!shotsTriggered) {
                        shooter.shoot();
                        shotsTriggered = true;
                    }

                    if (shotsTriggered && !shooter.isBusy()) {
                        shotsTriggered = false;
                        pathState = 6;
                    }
                }
                break;

            case 6:
                follower.followPath(paths.Path5);
                pathState = 7;
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    pathState = 9;
                }
                break;

            case 9: // Shoot after Path6
                if (!follower.isBusy()) {

                    if (!shotsTriggered) {
                        shooter.shoot();
                        shotsTriggered = true;
                    }

                    if (shotsTriggered && !shooter.isBusy()) {
                        shotsTriggered = false;
                        pathState = 10;
                    }
                }
                break;

            case 10:
                follower.followPath(paths.Path8);
                pathState = 11;
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    pathState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10);
                    pathState = 13;
                }
                break;

            case 13: // Shoot after Path9
                if (!follower.isBusy()) {

                    if (!shotsTriggered) {
                        shooter.shoot();
                        shotsTriggered = true;
                    }

                    if (shotsTriggered && !shooter.isBusy()) {
                        shotsTriggered = false;
                        pathState = 14;
                    }
                }
                break;
        }

        return pathState;
    }

}
    