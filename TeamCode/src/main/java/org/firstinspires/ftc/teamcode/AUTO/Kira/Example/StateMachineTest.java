package org.firstinspires.ftc.teamcode.AUTO.Kira.Example;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "StateMachineTest", group = "Tests")
public class StateMachineTest extends OpMode {

    private Follower follower;
    private PathChain drivePath;

    private Intake intake;
    private Shooter shooter;

    private enum State {
        DRIVE,
        SHOOT,
        DONE
    }

    private State state;

    private final Pose startPose = new Pose(24.5, 118.79, Math.toRadians(-90));
    private final Pose shootPose = new Pose(24.5, 84.09, Math.toRadians(-90));

    private boolean pathStarted = false; // для отслеживания запуска движения

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        intake = new Intake(hardwareMap.get(
                com.qualcomm.robotcore.hardware.DcMotor.class,
                Constants.intake));

        shooter = new Shooter(
                hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, Constants.shooter),
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, Constants.servoL),
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, Constants.servoR)
        );

        drivePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        follower.setPose(startPose);
        state = State.DRIVE;
    }

    @Override
    public void loop() {
        // Обновляем все подсистемы
        follower.update();
        intake.update();
        shooter.update();

        switch (state) {

            case DRIVE:
                // Запуск движения, intake и shooter только один раз
                if (!pathStarted) {
                    follower.followPath(drivePath, 0.5, true);
                    intake.start();      // Параллельно
                    shooter.spinUp();    // Параллельно
                    pathStarted = true;
                }

                // Ждём пока движение завершится
                if (!follower.isBusy()) {
                    state = State.SHOOT;
                }
                break;

            case SHOOT:
                // Запускаем выстрел, когда shooter готов
                if (shooter.isReady()) {
                    shooter.fire();
                }

                // Ждём завершения выстрела, чтобы перейти в DONE
                if (!shooter.isBusy()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                intake.stop();
                shooter.stop();
                break;
        }

        // Телеметрия для отладки
        telemetry.addData("State", state);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
