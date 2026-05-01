package org.firstinspires.ftc.teamcode.AUTO.Sasha;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous (name="Limelight Ty")
public class LimelightTy extends LinearOpMode {
    Limelight3A limelight;
    DcMotor leftFront, leftBack, rightFront, rightBack;

    static final double PI = 3.14159265;
    static final double WHEEL_DIAMETER = 10.4;
    static final double PULSES = 537.7;
    static final double PULSES_PER_CM = PULSES / (PI * WHEEL_DIAMETER);
    private IMU imu = null;
    final int GREEN_PIPELINE = 6;
    final int PURPLE_PIPELINE = 7;

    // Helper class to handle the lookup table logic
    private static class RampZone {
        double minX, maxX, minY, maxY, minArea, maxArea;

        RampZone(double tx, double ty, double taLow, double taHigh) {
            // Tolerances: tx +/- 1.0, ty +/- 0.5
            this.minX = tx - 1.0;
            this.maxX = tx + 1.0;
            this.minY = ty - 0.5;
            this.maxY = ty + 0.5;
            this.minArea = taLow;
            this.maxArea = taHigh;
        }

        public int getBallCount(double tx, double ty, double ta) {
            if (tx >= minX && tx <= maxX && ty >= minY && ty <= maxY) {
                if (ta > maxArea * 1.5) return 2; // Merged balls
                if (ta >= minArea * 0.7) return 1; // Single ball
            }
            return 0;
        }
    }

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

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        int greenCount = countArtifacts(GREEN_PIPELINE);
        int purpleCount = countArtifacts(PURPLE_PIPELINE);

        telemetry.addData("Final Green", greenCount);
        telemetry.addData("Final Purple", purpleCount);
        telemetry.addData("Total", greenCount + purpleCount);
        telemetry.update();

        sleep(4000);
    }

    private int countArtifacts(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);

        // Wait for hardware to switch and clear old results
        sleep(800);
        limelight.getLatestResult();

        ArrayList<RampZone> zones = new ArrayList<>();

        // Using Purple tx/ty coordinates for BOTH, but unique area values
        if (pipelineIndex == GREEN_PIPELINE) {
            zones.add(new RampZone(-24.6, -5.3, 0.250, 0.270));
            zones.add(new RampZone(-18.0, -3.0, 0.180, 0.230));
            zones.add(new RampZone(-13.0, -1.3, 0.210, 0.230));
            zones.add(new RampZone(-7.0,   0.2, 0.150, 0.180));
            zones.add(new RampZone(-1.0,   1.9, 0.150, 0.160));
            zones.add(new RampZone(8.5,    5.7, 0.180, 0.210));
            zones.add(new RampZone(13.8,   7.4, 0.170, 0.190));
            zones.add(new RampZone(19.4,   9.2, 0.130, 0.150));
            zones.add(new RampZone(24.5,  11.1, 0.170, 0.190));
        } else {
            zones.add(new RampZone(-24.6, -5.3, 0.240, 0.300));
            zones.add(new RampZone(-18.0, -3.0, 0.200, 0.230));
            zones.add(new RampZone(-13.0, -1.3, 0.170, 0.190));
            zones.add(new RampZone(-7.0,   0.2, 0.190, 0.210));
            zones.add(new RampZone(-1.0,   1.9, 0.200, 0.230));
            zones.add(new RampZone(8.5,    5.7, 0.180, 0.230));
            zones.add(new RampZone(13.8,   7.4, 0.170, 0.180));
            zones.add(new RampZone(19.4,   9.2, 0.140, 0.160));
            zones.add(new RampZone(24.5,  11.1, 0.160, 0.190));
        }

        long startTime = System.currentTimeMillis();
        int finalCount = 0;

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < 1000)) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() &&
                    result.getPipelineIndex() == pipelineIndex &&
                    result.getStaleness() < 100) {

                int frameCount = 0;
                List<ColorResult> blobs = result.getColorResults();

                for (ColorResult cr : blobs) {
                    for (RampZone zone : zones) {
                        int count = zone.getBallCount(cr.getTargetXDegrees(), cr.getTargetYDegrees(), cr.getTargetArea());
                        if (count > 0) {
                            frameCount += count;
                            break;
                        }
                    }
                }
                // Update with the most recent count from the correct pipeline
                finalCount = frameCount;
            }
            sleep(30);
        }
        return finalCount;
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}