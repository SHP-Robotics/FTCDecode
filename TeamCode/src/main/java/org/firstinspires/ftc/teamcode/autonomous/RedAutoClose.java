package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.autonomous.AutoPathsRedClose.PathState;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

@Disabled
@Autonomous(name = "Red Auto Close")
public class RedAutoClose extends BaseRobot {
    Telemetry dashboardTelemetry;

    private void shoot(double time, double power) {
        blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
        blockerSubsystem.update();
        intakeSubsystem.setPowerDirect(1.0);
        outtakeSubsystem.setPowerDirect(power);
        indexerSubsystem.setState(IndexerSubsystem.IndexerState.PULSE_OUT);

        long start = System.nanoTime();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            follower.update();

            turretSubsystem.update();
            indexerSubsystem.update();

            double elapsedTime = (System.nanoTime() - start) / 1E9;

            if (outtakeSubsystem.isBusy(power))
                intakeSubsystem.setPowerDirect(0.0);
            else
                intakeSubsystem.setPowerDirect(1.0);

            if (elapsedTime > time)
                break;

            dashboardTelemetry.addData("velocity", outtakeSubsystem.getVelocity());
            dashboardTelemetry.addData("expected velocity", outtakeSubsystem.getExpectedVelocity(power));
            dashboardTelemetry.update();

            telemetry.addData("velocity", outtakeSubsystem.getVelocity());
            telemetry.addData("expected velocity", outtakeSubsystem.getExpectedVelocity(power));
            telemetry.update();
        }

        blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
        blockerSubsystem.update();
        intakeSubsystem.setPowerDirect(0.0);
        indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUT);
        indexerSubsystem.update();
    }

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        super.initialize();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();


        AutoPathsRedClose.initializePaths();
        PathState pathState = PathState.FIRST_PATH;

        // create follower
        follower = Constants.createFollower(hardwareMap);
        MotorCortex.update();

        // zero follower
        follower.update();
        follower.setMaxPower(1.0);
        follower.setPose(new Pose(0, 0));
        follower.setStartingPose(new Pose(0, 0));
        follower.followPath(pathState.getPath());

        telemetry.addLine("ready");
        telemetry.update();

        blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
        blockerSubsystem.update();

        hoodSubsystem.setAngleDirect(0.60);

        turretSubsystem.setAcceptedTags(PestoFTCConfig.RED_TAGS);
        turretSubsystem.rezero();
//        turretSubsystem.setVisionOffset(3);

        // 68 degrees
        turretSubsystem.setTargetPosition(63 * 6.88);

        while (!isStarted() && !isStopRequested()) {
            MotorCortex.update();
            turretSubsystem.update();

            telemetry.addData("turret state", turretSubsystem.getState());
            telemetry.update();
        }

        if (isStopRequested())
            return;

        waitForStart();
        double power = -0.72;
//        outtakeSubsystem.setPowerDirect(power);

        long start = System.nanoTime();
        while (opModeIsActive() && !isStopRequested()) {

            MotorCortex.update();
            follower.update();

            double elapsedTime = (System.nanoTime() - start) / 1E9;

            if (pathState.getTimer() < elapsedTime) {
                switch (pathState) {
                    case FIRST_PATH:
//                        intakeSubsystem.setPowerDirect(1.0);

                        pathState = PathState.SECOND_PATH;
                        break;
                    case SECOND_PATH:
//                        intakeSubsystem.setPowerDirect(0.0);

                        pathState = PathState.THIRD_PATH;
                        break;
                    case THIRD_PATH:
//                        shoot(2.25, power);
//                        intakeSubsystem.setPowerDirect(1.0);

                        pathState = PathState.FOURTH_PATH;
                        break;
                    case FOURTH_PATH:
                        return;
                }

                follower.followPath(pathState.getPath());
                start = System.nanoTime();
            }

            turretSubsystem.update();

            dashboardTelemetry.addData("X", follower.getPose().getX());
            dashboardTelemetry.addData("Y", follower.getPose().getY());
            dashboardTelemetry.addData("R", follower.getPose().getHeading());
            dashboardTelemetry.addLine();
            dashboardTelemetry.addData("rpm", outtakeSubsystem.getRPM());
            dashboardTelemetry.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("R", follower.getPose().getHeading());
            telemetry.addLine();
            telemetry.addData("rpm", outtakeSubsystem.getRPM());
            telemetry.update();
        }

        turretSubsystem.visionPortal.close();
    }
}