
package org.firstinspires.ftc.teamcode.AUTO.Kira.Blue;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.AUTO.Kira.movements_9;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;



@Autonomous(name = "Blue_Base_12", group = "Autonomous")
@Configurable // Panels
public class Base_12 extends OpMode {
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
                                    new Pose(23.573, 119.218),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(24.560, 118.989),
                                    new Pose(24.333, 89.033)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.333, 89.033),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(25.869, 84.504),
                                    new Pose(24.894, 74.719)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.894, 74.719),

                                    new Pose(24.894, 65.848)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.894, 65.848),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.594, 94.576),
                                    new Pose(30.302, 78.164),
                                    new Pose(24.045, 53.170)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(-90))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.045, 53.170),

                                    new Pose(24.045, 40.079)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.045, 40.079),

                                    new Pose(48.594, 94.576)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(315))

                    .build();

            Path10 = follower.pathBuilder().addPath(
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

                    if (!shotsTriggered) {
                        shooter.shoot();
                        shotsTriggered = true;
                    }

                    if (shotsTriggered && !shooter.isBusy()) {
                        shotsTriggered = false;
                        pathState = 5;
                    }
                }
                break;

            case 5:
                follower.followPath(paths.Path4);
                pathState = 6;
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    pathState = 8;
                }
                break;

            case 8: // Shoot after Path6
                if (!follower.isBusy()) {

                    if (!shotsTriggered) {
                        shooter.shoot();
                        shotsTriggered = true;
                    }

                    if (shotsTriggered && !shooter.isBusy()) {
                        shotsTriggered = false;
                        pathState = 9;
                    }
                }
                break;

            case 9:
                follower.followPath(paths.Path7);
                pathState = 10;
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    pathState = 12;
                }
                break;

            case 12: // Shoot after Path9
                if (!follower.isBusy()) {

                    if (!shotsTriggered) {
                        shooter.shoot();
                        shotsTriggered = true;
                    }

                    if (shotsTriggered && !shooter.isBusy()) {
                        shotsTriggered = false;
                        pathState = 13;
                    }
                }
                break;

            case 13:
                follower.followPath(paths.Path10);
                pathState = 14;
                break;
        }

        return pathState;
    }

}
    