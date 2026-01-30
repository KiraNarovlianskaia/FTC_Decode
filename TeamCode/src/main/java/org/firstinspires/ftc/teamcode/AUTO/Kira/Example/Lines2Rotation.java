package org.firstinspires.ftc.teamcode.AUTO.Kira.Example;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name = "2 Lines + Rotation", group = "Tests")
public class Lines2Rotation extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS
    }
    PathState pathState;

    private final Pose startPose = new Pose(23.077534791252486,119.93638170974155, Math.toRadians(138));
    private final Pose shootPose = new Pose(59.54671968190855, 83.88071570576541, Math.toRadians(138));
    private final Pose endPose = new Pose(36.071570576540765, 58.49105367793243, Math.toRadians(90));

    private PathChain driveStartPosShootPos, driveShootPosEndPos;
    public void buildPaths() {
        //put in coordinates from starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }


    public void  statePathUpdate() {
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,0.5,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                // check is follower done it's path?
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    follower.followPath(driveShootPosEndPos,0.5, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                }
                break;
            case DRIVE_SHOOTPOS_ENDPOS:
                //all done
                if (!follower.isBusy()){
                    telemetry.addLine("Done all Paths");
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
