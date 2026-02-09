package org.firstinspires.ftc.teamcode.AUTO.Kira.Example;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Servos", group = "Tests")
public class ExampleServos extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private DcMotor intake;
    //private Servo servoL, servoR;


    public enum PathState {
        //START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        INTAKE_BALLS,
        DONE
    }
    PathState pathState;

    private final Pose startPose = new Pose(24.5,118.79, Math.toRadians(-90));
    private final Pose shootPose = new Pose(24.5, 84.09, Math.toRadians(-90));

    private PathChain driveStartPosShootPos;
    public void buildPaths() {
        //put in coordinates from starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }


    public void  statePathUpdate() {
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, 0.5,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                // check is follower done it's path?
                if (!follower.isBusy()){
                    // TODO add logic to shooter
                    telemetry.addLine("Done Path l");
                    setPathState(PathState.INTAKE_BALLS);
                }
                break;
            case INTAKE_BALLS:
                if (!follower.isBusy()) {
                    intakeOn();
                    // собираем шары 1.5 сек
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        intakeOff();
                        setPathState(PathState.DONE);
                    }
                }
                break;
            case DONE:
                // ничего не делаем
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
        // ===== DC MOTORS =====
        intake = hardwareMap.get(DcMotor.class, Constants.intake);

        intake.setDirection(DcMotor.Direction.FORWARD);

        buildPaths();
        follower.setPose(startPose);

    }

    public void intakeOn() {
        intake.setPower(Constants.intake_power);
    }
    public void intakeOff() {
        intake.setPower(0);
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
