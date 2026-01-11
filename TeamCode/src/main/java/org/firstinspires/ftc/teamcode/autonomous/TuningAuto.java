package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@Config
@Autonomous(name = "Tuning Auto")
public class TuningAuto extends BaseRobot {
    public static double endpoint_kp = 0.00;
    public static double heading_kp = 0.5;
    public static double deceleration = 58;
    public static double static_power = 0.1;

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        super.initialize();

        mecanumController.setStaticPower(static_power);

        PathContainer MOVE = new PathContainer.PathContainerBuilder()
                .setStartPosition(new Pose(0, 0))
                .setIncrement(0.01)
                .addCurve(new BezierCurve(
                        new Pose[]{
                                new Pose(0, 0),
                                new Pose(30, 0)
                        }
                ), new ParametricHeading(v -> 0.0))
                .build();

        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                FrontalLobe.driveController,
                FrontalLobe.tracker,
                MOVE
        )
                .setDeceleration(deceleration)
                .setLookAhead(1.0)
                .setSpeed(0.75)
                .setHeadingPID(new PID(heading_kp, 0, 0))
                .setEndpointPID(new PID(endpoint_kp, 0, 0))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            tracker.update();

            boolean isStatic = tracker.getRobotVelocity().getMagnitude() < 1.0;
            mecanumController.setIsStatic(isStatic);

            pathFollower.update();

            telemetry.addData("X", tracker.getCurrentPosition().getX());
            telemetry.addData("Y", tracker.getCurrentPosition().getY());
            telemetry.addData("R", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
}
