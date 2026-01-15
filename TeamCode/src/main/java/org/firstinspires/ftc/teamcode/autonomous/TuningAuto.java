package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.geometries.BezierCurve;
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
    public static double endpoint_kp = 0;
    public static double heading_kp = 0;
    public static double deceleration = 0.01;
    public static double speed = 0.5;
    public static double look_ahead = 1;

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        super.initialize();

        mecanumController.setStaticPower(PestoFTCConfig.STATIC_DRIVE);

        PathContainer MOVE = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(new BezierCurve(
                        new Pose[]{
                                new Pose(0, 0, 0.0),
                                new Pose(0, 30, 0.0)
                        }
                ))
                .build();

        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                FrontalLobe.driveController,
                FrontalLobe.tracker,
                MOVE,
                0.2,
                0.05,
                0.2
        )
                .setDeceleration(deceleration)
                .setLookAhead(look_ahead)
                .setSpeed(speed)
                .setHeadingPID(new PID(heading_kp, 0, 0))
                .setEndpointPID(new PID(endpoint_kp, 0, 0))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            tracker.update();

//            boolean isStatic = tracker.getRobotVelocity().getMagnitude() < 1.0;
//            mecanumController.setIsStatic(isStatic);

            pathFollower.update();

            telemetry.addData("decelerating", pathFollower.isDecelerating());
            telemetry.addData("X", tracker.getCurrentPosition().getX());
            telemetry.addData("Y", tracker.getCurrentPosition().getY());
            telemetry.addData("R", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
}
