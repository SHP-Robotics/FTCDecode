package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.autonomous.AutoPathsRedClose.PathState;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;

@Autonomous(name = "Red Auto Far")
public class RedAutoFar extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        super.initialize();

        AutoPathsRedClose.initializePaths();
        PathState pathState = PathState.FIRST_PATH;

        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("ready");
        telemetry.update();

        blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
        blockerSubsystem.update();

        waitForStart();

        MotorCortex.update();
        follower.update();

        follower.setMaxPower(1.0);
        follower.setPose(new Pose(0, 0));
        follower.setStartingPose(new Pose(0, 0));
        follower.followPath(pathState.getPath());

        long start = System.nanoTime();

        while (opModeIsActive() && !isStopRequested()) {

            MotorCortex.update();
            follower.update();

            double elapsedTime = (System.nanoTime() - start) / 1E9;

            if (pathState.getTimer() < elapsedTime) {
                switch (pathState) {
                    case FIRST_PATH:
                        intakeSubsystem.setPowerDirect(1.0);

                        pathState = PathState.SECOND_PATH;
                        break;
                    case SECOND_PATH:
                        intakeSubsystem.setPowerDirect(0.0);

                        pathState = PathState.THIRD_PATH;
                        break;
                    case THIRD_PATH:
                        intakeSubsystem.setPowerDirect(1.0);

                        pathState = PathState.FOURTH_PATH;
                        break;
                    case FOURTH_PATH:
                        intakeSubsystem.setPowerDirect(0.0);

                        pathState = PathState.FIFTH_PATH;
                        break;
                    case FIFTH_PATH:
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