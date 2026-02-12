package org.firstinspires.ftc.teamcode.AUTO.Sensors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
@Disabled
@Autonomous(name = "AprilTag ID Reader", group = "Vision")
public class Camera extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        // Инициализация Limelight3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Выбираем pipeline, который настроен на AprilTag (0 по умолчанию)
        limelight.pipelineSwitch(8);

        // Настраиваем частоту обновления
        limelight.setPollRateHz(30);

        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();

        limelight.start();  // Запускаем камеру

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult fid : fiducials) {
                        int tagId = fid.getFiducialId();
                        telemetry.addData("AprilTag ID", tagId);
                    }
                } else {
                    telemetry.addData("AprilTag", "No tag detected");
                }

            } else {
                telemetry.addData("Limelight", "No valid result");
            }

            telemetry.update();
        }
    }
}
