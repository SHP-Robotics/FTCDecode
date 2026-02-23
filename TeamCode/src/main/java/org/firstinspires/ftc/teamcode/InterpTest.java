package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "Interp Test", group = "Concept")
public class InterpTest extends LinearOpMode {
    public static double shooter_power = 0.0;
    public static double hood_angle = 0.0;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        HoodSubsystem hoodSubsystem = new HoodSubsystem();
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

        PID pidController = new PID(0.02, 0, 0);

        TurretSubsystem turretSubsystem = new TurretSubsystem();
        turretSubsystem.setState(TurretSubsystem.TurretState.MANUAL);
        boolean scanningLeft = true;

        AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            MotorCortex.update();

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            boolean detected = false;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata == null)
                    continue;
                detected = true;

                double turretDegrees = turretSubsystem.getPosition() * 45 / 310;
                double power = pidController.getOutput(turretDegrees, turretDegrees + detection.ftcPose.bearing);
                scanningLeft = power < 0;

                if (turretSubsystem.getPosition() < -620) {
                    power = Math.max(power, 0);
                }

                if (turretSubsystem.getPosition() > 1860) {
                    power = Math.min(power, 0);
                }

                turretSubsystem.setPower(power);

                telemetry.addData("position", turretSubsystem.getPosition());
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addData("Bearing", detection.ftcPose.bearing);
                telemetry.addData("Distance", detection.ftcPose.y);
            }

            if (!detected) {
                if (turretSubsystem.getPosition() < -620) {
                    scanningLeft = false;
                }

                if (turretSubsystem.getPosition() > 1860) {
                    scanningLeft = true;
                }

                turretSubsystem.setPower(scanningLeft ? -0.2 : 0.2);
            }

            if (gamepad1.left_trigger > 0.5) {
                outtakeSubsystem.setPowerDirect(shooter_power);
                hoodSubsystem.setAngleDirect(hood_angle);
            } else {
                outtakeSubsystem.setPowerDirect(0.0);
            }

            if (gamepad1.right_trigger > 0.5) {
                intakeSubsystem.setPowerDirect(1);
            } else if (gamepad1.a) {
                intakeSubsystem.setPowerDirect(-1);
            } else {
                intakeSubsystem.setPowerDirect(0);
            }

            telemetry.update();

            turretSubsystem.update();
        }

        visionPortal.close();
    }
}