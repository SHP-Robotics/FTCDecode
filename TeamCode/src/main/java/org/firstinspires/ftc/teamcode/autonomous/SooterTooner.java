package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "Sooter Tooner")
public class SooterTooner extends BaseRobot {
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

        telemetry.addLine("ready");
        telemetry.update();

        blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
        blockerSubsystem.update();

        hoodSubsystem.setAngleDirect(0.60);

        turretSubsystem.setAcceptedTags(PestoFTCConfig.RED_TAGS);
        turretSubsystem.rezero();
        turretSubsystem.setVisionOffset(2);

        // 60 degrees
        turretSubsystem.setPosition(70 * 6.88);

        while (!isStarted() && !isStopRequested()) {
            MotorCortex.update();
            turretSubsystem.update();

            if (turretSubsystem.getState() == TurretSubsystem.TurretState.CUSTOM_POSITION && turretSubsystem.getAprilTagBearing() != null)
                    turretSubsystem.setState(TurretSubsystem.TurretState.AUTO);

            telemetry.addData("turret state", turretSubsystem.getState());
            telemetry.update();
        }

        waitForStart();
        double power = -0.675;
        outtakeSubsystem.setPowerDirect(power);

        long start = System.nanoTime();
        while (opModeIsActive() && !isStopRequested() && (System.nanoTime() - start) / 1E9 < 2) {
            MotorCortex.update();
            turretSubsystem.update();

            if (turretSubsystem.getState() == TurretSubsystem.TurretState.CUSTOM_POSITION && turretSubsystem.getAprilTagBearing() != null)
                turretSubsystem.setState(TurretSubsystem.TurretState.AUTO);

            telemetry.addData("turret state", turretSubsystem.getState());
            telemetry.update();
        }

        shoot(30, power);

        turretSubsystem.visionPortal.close();
    }
}