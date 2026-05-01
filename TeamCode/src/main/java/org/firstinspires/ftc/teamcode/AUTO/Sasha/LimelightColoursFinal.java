package org.firstinspires.ftc.teamcode.AUTO.Sasha;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.LLStatus;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
@Disabled
@Autonomous (name="Limelight Colours Final")
public class LimelightColoursFinal extends LinearOpMode {
    Limelight3A limelight;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    static final double PI = 3.14159265;
    static final double WHEEL_DIAMETER = 10.4;
    static final double PULSES = 537.7;
    static final double PULSES_PER_CM = PULSES / (PI * WHEEL_DIAMETER);
    private IMU imu = null;


    final int APRILTAG_PIPELINE = 8;
    final int GREEN_PIPELINE = 6;
    final int PURPLE_PIPELINE = 7;

    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        limelight.setPollRateHz(100);
        limelight.start();


        waitForStart();
        int greenCount = countArtifacts(GREEN_PIPELINE);

        int purpleCount = countArtifacts(PURPLE_PIPELINE);

        telemetry.addData("Green Count", greenCount);
        telemetry.addData("Purple Count", purpleCount);
        telemetry.addData("Total Artifacts", greenCount + purpleCount);
        telemetry.update();

        sleep(4000);
    }

    public void moveForward(double speed, double distance) {

        resetEncoders();

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSES_PER_CM * distance)
            ;

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }

    public void moveRotate(double speed, double angle) {

        imu.resetYaw();

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(-speed);
        rightBack.setPower(-speed);

        while (opModeIsActive() &&
                Math.abs(getHeading()) < Math.abs(angle)) {
            idle();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }

    public void moveSide(double speed, double distance) {

        resetEncoders();

        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && Math.abs(leftFront.getCurrentPosition()) < PULSES_PER_CM * distance)
            ;

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        sleep(500);
    }


    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private int countArtifacts(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);

        // 1. Give the camera a clear head-start to switch hardware modes
        sleep(400);

        long startTime = System.currentTimeMillis();
        int bestCount = 0;

        // Loop for 1.2 seconds to "watch" the balls and pick the most stable count
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < 1200)) {
            LLResult result = limelight.getLatestResult();

            // 2. CRITICAL CHECKS:
            // - Is the result valid?
            // - Is it actually from the pipeline we just switched to?
            // - Is it "Fresh"? (Staleness < 100ms)
            if (result != null && result.isValid() &&
                    result.getPipelineIndex() == pipelineIndex &&
                    result.getStaleness() < 100) {

                List<ColorResult> blobs = result.getColorResults();
                int currentFrameCount = 0;
                double oneBallArea = 0.191; // Your baseline

                for (ColorResult cr : blobs) {
                    double area = cr.getTargetArea();

                    // Use Math.round to capture overlapping balls
                    int ballsInBlob = (int) Math.round(area / oneBallArea);
                    if (ballsInBlob < 1) ballsInBlob = 1;

                    currentFrameCount += ballsInBlob;
                }

                // Update bestCount if this frame saw more (handles flickering)
                if (currentFrameCount > bestCount) {
                    bestCount = currentFrameCount;
                }
            }
            sleep(30); // Faster polling for better accuracy
        }

        telemetry.addData("Vision Pipeline " + pipelineIndex, "Final Count: " + bestCount);
        telemetry.update();
        return bestCount;
    }
}