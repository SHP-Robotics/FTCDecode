package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.PestoFTCConfig.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.autonomous.AutoPathsRedFar.PathState;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

@Autonomous(name = "Red Auto Far")
public class RedAutoFar extends BaseRobot {
    Telemetry dashboardTelemetry;

    private void shoot(double timePrecise, double timeRapid, double power, double position) {
        blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
        blockerSubsystem.update();
        intakeSubsystem.setPowerDirect(1.0);
        outtakeSubsystem.setPowerDirect(power);
        turretSubsystem.setTargetPosition(position);
        indexerSubsystem.setState(IndexerSubsystem.IndexerState.PULSE_OUT);

//        long start = System.nanoTime();
//        while (opModeIsActive() && !isStopRequested()) {
//            MotorCortex.update();
//            follower.update();
//
//            turretSubsystem.update();
//            indexerSubsystem.update();
//            outtakeSubsystem.update();
//
//            double elapsedTime = (System.nanoTime() - start) / 1E9;
//
//            if (outtakeSubsystem.isBusy(power))
//                intakeSubsystem.setPowerDirect(0.0);
//            else
//                intakeSubsystem.setPowerDirect(1.0);
//
//            if (elapsedTime > timePrecise)
//                break;
//
//            dashboardTelemetry.addData("velocity", outtakeSubsystem.getVelocity());
//            dashboardTelemetry.addData("expected velocity", outtakeSubsystem.getExpectedVelocity(0));
//            dashboardTelemetry.addData("VelocityMax", 2500);
//            dashboardTelemetry.addData("VelocityMin", 0);
//
////            dashboardTelemetry.addData("turret position", turretSubsystem.getPosition());
////            dashboardTelemetry.addData("target position", turretSubsystem.getTargetPosition());
//
//            if (turretSubsystem.lastAprilTag != null)
//                dashboardTelemetry.addData("turret bearing", turretSubsystem.lastAprilTag.bearing);
//            dashboardTelemetry.update();
//        }

        while (opModeIsActive() && !isStopRequested() && outtakeSubsystem.isBusy(power)) {
            MotorCortex.update();
            follower.update();

            turretSubsystem.update();
            indexerSubsystem.update();
            outtakeSubsystem.update();
        }

        long start = System.nanoTime();
        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            follower.update();

            turretSubsystem.update();
            indexerSubsystem.update();
            outtakeSubsystem.update();

            double elapsedTime = (System.nanoTime() - start) / 1E9;

            intakeSubsystem.setPowerDirect(1.0);

            if (elapsedTime > timeRapid)
                break;

            dashboardTelemetry.addData("velocity", outtakeSubsystem.getVelocity());
            dashboardTelemetry.addData("expected velocity", outtakeSubsystem.getExpectedVelocity(0));
            dashboardTelemetry.addData("VelocityMax", 2500);
            dashboardTelemetry.addData("VelocityMin", 0);

//            dashboardTelemetry.addData("turret position", turretSubsystem.getPosition());
//            dashboardTelemetry.addData("target position", turretSubsystem.getTargetPosition());

            if (turretSubsystem.lastAprilTag != null)
                dashboardTelemetry.addData("turret bearing", turretSubsystem.lastAprilTag.bearing);
            dashboardTelemetry.update();
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


        AutoPathsRedFar.initializePaths();
        PathState pathState = PathState.SHOOT_PATH;

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
        turretSubsystem.setDetectAprilTag(true);
        turretSubsystem.setVisionOffset(3.5);

        // 68 degrees
        double position = 60 * 6.88;
        turretSubsystem.setTargetPosition(position);

        while (!isStarted() && !isStopRequested()) {
            MotorCortex.update();
            turretSubsystem.update();

            if (turretSubsystem.detected)
                turretSubsystem.setRecenterAfter(false);

            telemetry.addData("detected", turretSubsystem.lastAprilTag != null);
            telemetry.update();

            dashboardTelemetry.addData("velocity", outtakeSubsystem.getVelocity());
            dashboardTelemetry.addData("expected velocity", outtakeSubsystem.getExpectedVelocity(0));
            dashboardTelemetry.addData("VelocityMax", 2500);
            dashboardTelemetry.addData("VelocityMin", 0);

//            dashboardTelemetry.addData("turret position", turretSubsystem.getPosition());
//            dashboardTelemetry.addData("target position", turretSubsystem.getTargetPosition());

            if (turretSubsystem.lastAprilTag != null)
                dashboardTelemetry.addData("turret bearing", turretSubsystem.lastAprilTag.bearing);
            dashboardTelemetry.update();
        }

        if (isStopRequested())
            return;

        waitForStart();
        turretSubsystem.setVisionOffset(4);
        double power = -0.78;
        outtakeSubsystem.setPower(power);

        long start = System.nanoTime();
        while (opModeIsActive() && !isStopRequested()) {

            MotorCortex.update();
            follower.update();

            double elapsedTime = (System.nanoTime() - start) / 1E9;

            if (pathState.getTimer() < elapsedTime) {
                switch (pathState) {
                    case SHOOT_PATH:
                        shoot(2, 3, power, position);

                        pathState = PathState.FIRST_PATH;
                        break;
                    case FIRST_PATH:
                        intakeSubsystem.setPowerDirect(1.0);

                        pathState = PathState.SECOND_PATH;
                        break;
                    case SECOND_PATH:
                        intakeSubsystem.setPowerDirect(0.0);

                        pathState = PathState.THIRD_PATH;
                        break;
                    case THIRD_PATH:
                        shoot(2, 3, power, position);
                        intakeSubsystem.setPowerDirect(1.0);

                        pathState = PathState.FOURTH_PATH;
                        break;
                    case FOURTH_PATH:
//                        intakeSubsystem.setPowerDirect(0.0);

                        pathState = PathState.FIFTH_PATH;
                        break;
                    case FIFTH_PATH:
                        shoot(2, 3, power, position);
                        intakeSubsystem.setPowerDirect(1.0);

                        pathState = PathState.SIXTH_PATH;
                        break;
                    case SIXTH_PATH:
//                        intakeSubsystem.setPowerDirect(0.0);

                        pathState = PathState.SEVENTH_PATH;
                        break;
                    case SEVENTH_PATH:
                        shoot(2, 3, power, position);
                        intakeSubsystem.setPowerDirect(1.0);

                        pathState = PathState.EIGHTH_PATH;
                        break;
                    case EIGHTH_PATH:
                        return;
                }

                follower.followPath(pathState.getPath());
                start = System.nanoTime();
            }

//            turretSubsystem.update();
            turretSubsystem.setPower(0.0);
            outtakeSubsystem.update();

            dashboardTelemetry.addData("velocity", outtakeSubsystem.getVelocity());
            dashboardTelemetry.addData("expected velocity", outtakeSubsystem.getExpectedVelocity(0));
            dashboardTelemetry.addData("VelocityMax", 2500);
            dashboardTelemetry.addData("VelocityMin", 0);

//            dashboardTelemetry.addData("turret position", turretSubsystem.getPosition());
//            dashboardTelemetry.addData("target position", turretSubsystem.getTargetPosition());

            if (turretSubsystem.lastAprilTag != null)
                dashboardTelemetry.addData("turret bearing", turretSubsystem.lastAprilTag.bearing);
            dashboardTelemetry.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("R", follower.getPose().getHeading());
            telemetry.update();
        }

        turretSubsystem.visionPortal.close();
    }
}