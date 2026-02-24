package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.autonomous.AutoPathsRedClose.PathState;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@Autonomous(name = "Red Auto Far")
public class RedAutoFar extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        super.initialize();

        AutoPathsRedClose.initializePaths();
        PathState pathState = PathState.FIRST_PATH;

        follower.setMaxPower(0.5);
        follower.setStartingPose(new Pose(0, 0));

        waitForStart();

        follower.setPose(new Pose(0, 0));
        follower.followPath(pathState.getPath());

        long start = System.nanoTime();

        while (opModeIsActive() && !isStopRequested()) {

            MotorCortex.update();
            follower.update();

            double elapsedTime = (System.nanoTime() - start) / 1E9;

            if (pathState.getTimer() < elapsedTime) {
                switch (pathState) {
                    case FIRST_PATH:
                        pathState = PathState.SECOND_PATH;
                        break;
                    case SECOND_PATH:
                        pathState = PathState.THIRD_PATH;
                        break;
                    case THIRD_PATH:
                        pathState = PathState.FOURTH_PATH;
                        break;
                    case FOURTH_PATH:
                        pathState = PathState.FIFTH_PATH;
                        break;
                    case FIFTH_PATH:
                        pathState = PathState.SIXTH_PATH;
                        break;
                    case SIXTH_PATH:
                        pathState = PathState.SEVENTH_PATH;
                        break;
                    case SEVENTH_PATH:
                        pathState = PathState.EIGHTH_PATH;
                        break;
                    case EIGHTH_PATH:
                        return;
                }

                follower.followPath(pathState.getPath());
                start = System.nanoTime();
            }

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("R", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}