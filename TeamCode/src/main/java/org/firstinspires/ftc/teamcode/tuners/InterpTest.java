package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Disabled
@Config
@TeleOp(name = "Interp Test", group = "Concept")
public class InterpTest extends LinearOpMode {
    public static double shooter_power = -0.80;
    public static double hood_angle = 0.7;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        HoodSubsystem hoodSubsystem = new HoodSubsystem();
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

        TurretSubsystem turretSubsystem = new TurretSubsystem();
        turretSubsystem.setState(TurretSubsystem.TurretState.AUTO);

        IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

        waitForStart();

        indexerSubsystem.setState(IndexerSubsystem.IndexerState.PULSE_OUT);

        while (opModeIsActive()) {
            MotorCortex.update();

            telemetry.addData("distance", turretSubsystem.getAprilTagDistance());
            telemetry.update();

            if (gamepad1.left_trigger > 0.5) {
                outtakeSubsystem.setPowerDirect(shooter_power);
                hoodSubsystem.setAngleDirect(hood_angle);
            } else {
                outtakeSubsystem.setPowerDirect(0.0);
            }

            if (gamepad1.right_trigger > 0.5 && !outtakeSubsystem.isBusy(shooter_power)) {
                intakeSubsystem.setPowerDirect(1);
            } else if (gamepad1.a) {
                intakeSubsystem.setPowerDirect(-1);
            } else {
                intakeSubsystem.setPowerDirect(0);
            }

            telemetry.update();

            indexerSubsystem.update();
            turretSubsystem.update();
        }

        turretSubsystem.visionPortal.close();
    }
}