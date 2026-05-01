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
@Autonomous (name="Limelight Tx")
public class LimelightTx extends LinearOpMode {
    Limelight3A limelight;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    static final double PI = 3.1415926;
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

    // Helper class to handle the lookup table logic
    private static class RampZone {
        double minX, maxX, minY, maxY, minArea, maxArea;

        RampZone(double tx, double ty, double taLow, double taHigh) {
            // Tightened tolerances as requested: tx +/- 1.0, ty +/- 0.5
            this.minX = tx - 1.0;
            this.maxX = tx + 1.0;
            this.minY = ty - 0.5;
            this.maxY = ty + 0.5;
            this.minArea = taLow;
            this.maxArea = taHigh;
        }

        /**
         * Returns 0 if not in zone, 1 if area matches a single ball,
         * 2 if area suggests two balls merged, etc.
         */
        public int getBallCount(double tx, double ty, double ta) {
            if (tx >= minX && tx <= maxX && ty >= minY && ty <= maxY) {
                // Determine count based on area within this specific zone
                // If the area is significantly larger than the max expected, it's 2 balls
                if (ta > maxArea * 1.5) {
                    return 2;
                }
                // If the area is at least 70% of our minimum, count it as 1
                if (ta >= minArea * 0.7) {
                    return 1;
                }
            }
            return 0;
        }
    }

    private int countArtifacts(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);

        // FIX FOR GHOSTING:
        // We wait and then "poll" the results a few times to clear the old pipeline's cache
        sleep(600);
        limelight.getLatestResult();

        java.util.ArrayList<RampZone> zones = new java.util.ArrayList<>();

        // Using your PURPLE tx/ty coordinates for BOTH pipelines now
        // But keeping the GREEN area values for the green pipeline logic
        if (pipelineIndex == GREEN_PIPELINE) {
            zones.add(new RampZone(-24.6, -5.3, 0.250, 0.270)); // Pos 1
            zones.add(new RampZone(-18.0, -3.0, 0.180, 0.230)); // Pos 2
            zones.add(new RampZone(-13.0, -1.3, 0.210, 0.230)); // Pos 3
            zones.add(new RampZone(-7.0,   0.2, 0.150, 0.180)); // Pos 4
            zones.add(new RampZone(-1.0,   1.9, 0.150, 0.160)); // Pos 5
            zones.add(new RampZone(8.5,    5.7, 0.180, 0.210)); // Pos 6
            zones.add(new RampZone(13.8,   7.4, 0.170, 0.190)); // Pos 7
            zones.add(new RampZone(19.4,   9.2, 0.130, 0.150)); // Pos 8
            zones.add(new RampZone(24.5,  11.1, 0.170, 0.190)); // Pos 9
        } else { // PURPLE
            zones.add(new RampZone(-24.6, -5.3, 0.240, 0.300)); // Pos 1
            zones.add(new RampZone(-18.0, -3.0, 0.200, 0.230)); // Pos 2
            zones.add(new RampZone(-13.0, -1.3, 0.170, 0.190)); // Pos 3
            zones.add(new RampZone(-7.0,   0.2, 0.190, 0.210)); // Pos 4
            zones.add(new RampZone(-1.0,   1.9, 0.200, 0.230)); // Pos 5
            zones.add(new RampZone(8.5,    5.7, 0.180, 0.230)); // Pos 6
            zones.add(new RampZone(13.8,   7.4, 0.170, 0.180)); // Pos 7
            zones.add(new RampZone(19.4,   9.2, 0.140, 0.160)); // Pos 8
            zones.add(new RampZone(24.5,  11.1, 0.160, 0.190)); // Pos 9
        }

        long startTime = System.currentTimeMillis();
        int finalCount = 0;

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < 1000)) {
            LLResult result = limelight.getLatestResult();

            // Added check: result.getPipelineIndex() == pipelineIndex
            // This ensures we ONLY count if the camera confirms it is on the new color
            if (result != null && result.isValid() &&
                    result.getPipelineIndex() == pipelineIndex &&
                    result.getStaleness() < 100) {

                List<ColorResult> blobs = result.getColorResults();
                int frameCount = 0;

                for (ColorResult cr : blobs) {
                    double tx = cr.getTargetXDegrees();
                    double ty = cr.getTargetYDegrees();
                    double ta = cr.getTargetArea();

                    int foundCount = 0;
                    for (RampZone zone : zones) {
                        int count = zone.getBallCount(tx, ty, ta);
                        if (count > 0) {
                            foundCount = count;
                            break;
                        }
                    }
                    frameCount += foundCount;
                }

                // If a frame shows a count, we trust it if it's the highest seen
                if (frameCount > finalCount) finalCount = frameCount;
            }
            sleep(30);
        }
        return finalCount;
    }
}