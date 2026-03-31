//package org.firstinspires.ftc.teamcode.AUTO.Kira.Example;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "Example Servos", group = "Tests")
//public class ExampleServos extends OpMode {
//
//    private Follower follower;
//    private Timer stateTimer;
//
//    private DcMotor intake, shooter;
//    private Servo servoL, servoR;
//
//    private boolean pathStarted = false;
//    private boolean shootingStarted = false;
//
//    private final Pose startPose = new Pose(24.5, 118.79, Math.toRadians(-90));
//    private final Pose shootPose = new Pose(24.5, 84.09, Math.toRadians(-90));
//
//    private PathChain drivePath;
//
//    private enum State {
//        DRIVE,
//        INTAKE,
//        SHOOT,
//        DONE
//    }
//
//    private State state;
//
//    // ================= BUILD PATH =================
//    private void buildPaths() {
//        drivePath = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, shootPose))
//                .setLinearHeadingInterpolation(
//                        startPose.getHeading(),
//                        shootPose.getHeading()
//                )
//                .build();
//    }
//
//    // ================= INIT =================
//    @Override
//    public void init() {
//
//        follower = Constants.createFollower(hardwareMap);
//        stateTimer = new Timer();
//
//        intake = hardwareMap.get(DcMotor.class, Constants.intake);
//        shooter = hardwareMap.get(DcMotor.class, Constants.shooter);
//        servoL = hardwareMap.get(Servo.class, Constants.servoL);
//        servoR = hardwareMap.get(Servo.class, Constants.servoR);
//
//        intake.setDirection(DcMotor.Direction.FORWARD);
//        shooter.setDirection(DcMotor.Direction.REVERSE);
//
//        intake.setPower(0);
//        shooter.setPower(0);
//
//        servoL.setPosition(Constants.servo_init);
//        servoR.setPosition(Constants.servo_init);
//
//        buildPaths();
//        follower.setPose(startPose);
//
//        state = State.DRIVE;
//    }
//
//    // ================= LOOP =================
//    @Override
//    public void loop() {
//
//        follower.update();
//
//        switch (state) {
//
//            // ---------- DRIVE ----------
//            case DRIVE:
//                if (!pathStarted) {
//                    follower.followPath(drivePath, 0.5, true);
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy()) {
//                    stateTimer.resetTimer();
//                    state = State.INTAKE;
//                }
//                break;
//
//            // ---------- INTAKE ----------
//            case INTAKE:
//                if (stateTimer.getElapsedTimeSeconds() < 1.5) {
//                    intake.setPower(Constants.intake_power);
//                } else {
//                    intake.setPower(0);
//                    stateTimer.resetTimer();
//                    state = State.SHOOT;
//                }
//                break;
//
//            // ---------- SHOOT ----------
//            case SHOOT:
//
//                if (!shootingStarted) {
//                    shooter.setPower(1.0);
//                    shootingStarted = true;
//                    stateTimer.resetTimer();
//                }
//
//                double t = stateTimer.getElapsedTimeSeconds();
//
//                if (t >= 3.0) {
//                    servoL.setPosition(Constants.servo_shoot);
//                    servoR.setPosition(Constants.servo_shoot);
//                }
//
//                if (t >= 3.5) {
//                    shooter.setPower(0);
//                    servoL.setPosition(Constants.servo_init);
//                    servoR.setPosition(Constants.servo_init);
//                    state = State.DONE;
//                }
//                break;
//
//            // ---------- DONE ----------
//            case DONE:
//                break;
//        }
//
//        telemetry.addData("State", state);
//        telemetry.addData("Timer", stateTimer.getElapsedTimeSeconds());
//        telemetry.addData("X", follower.getPose().getX());
//        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("Heading", follower.getPose().getHeading());
//    }
//}
