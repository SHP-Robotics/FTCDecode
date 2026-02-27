package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@Disabled
@TeleOp(name = "Concept: AprilTag", group = "Concept")
public class ConceptAprilTag extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        PID pidController = new PID(0.02, 0, 0);

        TurretSubsystem turretSubsystem = new TurretSubsystem();
        turretSubsystem.setState(TurretSubsystem.TurretState.MANUAL);
        boolean scanningLeft = true;

//        AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessors(aprilTag)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .build();

        AprilTagProcessor aprilTag = turretSubsystem.aprilTag;
        VisionPortal visionPortal = turretSubsystem.visionPortal;

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        turretSubsystem.rezero();

        if (opModeIsActive()) {
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

                telemetry.update();

                turretSubsystem.update();

//                if (gamepad1.dpad_down) {
//                    visionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    visionPortal.resumeStreaming();
//                }
            }
        }

        visionPortal.close();
    }
}