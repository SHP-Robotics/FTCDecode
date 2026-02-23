package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "New Red Drive")
public class NewRedDrive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        boolean revving = false;
        boolean outtaking = false;

        super.initialize();

        Pose xModePosition = new Pose(0, 0, 0);

        PID pidController = new PID(0.02, 0, 0);

        turretSubsystem.setState(TurretSubsystem.TurretState.MANUAL);
        boolean scanningLeft = true;

        AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        double cam_distance = 0.0;

        waitForStart();

        turretSubsystem.rezero();

        while (opModeIsActive() && !isStopRequested()) {
            PestoFTCConfig.recalculate_interpolators();

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            tracker.update();

            boolean detected = false;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata == null)
                    continue;

                if (detection.id != 24)
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
                cam_distance = detection.ftcPose.y;
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





            boolean isStatic = tracker.getRobotVelocity().getMagnitude() < 1.0;
            mecanumController.setIsStatic(isStatic);

            if (gamepad1.x) {
                brake = !brake;
                mecanumController.setZeroPowerBehavior(brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if (gamepad1.x) {
                tracker.reset();
                teleOpController.resetIMU();
            }

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            boolean intaking = gamepad1.right_trigger > 0.05;
            outtaking = !intaking && gamepadInterface1.getTimeDown(GamepadKey.LEFT_TRIGGER) > 0.3;
            revving = (revving != gamepadInterface1.isClicked(GamepadKey.LEFT_TRIGGER, 0.3)) || outtaking;
            telemetry.addData("outtaking", outtaking);
            telemetry.addData("revving", revving);
            telemetry.addData("held", gamepadInterface1.isHeld(GamepadKey.LEFT_TRIGGER, 0.3));
            telemetry.addData("clicked", gamepadInterface1.isClicked(GamepadKey.LEFT_TRIGGER, 0.3));
            boolean rejecting = !intaking && !outtaking && gamepad1.a;
            boolean neutralizing = !intaking && !outtaking && !rejecting;

            if (state == RobotState.OUTTAKE) {
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
            }

            if (gamepad1.dpad_left) {
                while (opModeIsActive() && !isStopRequested() && gamepad1.dpad_left) {
                    turretSubsystem.setPower(0.3);
                }
                turretSubsystem.setPower(0.0);
            }

            if (gamepad1.dpad_right) {
                while (opModeIsActive() && !isStopRequested() && gamepad1.dpad_right) {
                    turretSubsystem.setPower(-0.3);
                }
                turretSubsystem.setPower(0.0);
            }

            if (gamepad1.dpad_up && gamepad1.y) {
                turretSubsystem.rezero();
            } else if (gamepad1.y) {
//                PathChain pathChain = PestoFTCConfig.follower.pathBuilder()
//                        .addPath(new BezierLine(PestoFTCConfig.follower.getPose(), xModePosition))
//                        .setLinearHeadingInterpolation(PestoFTCConfig.follower.getPose().getHeading(), xModePosition.getHeading())
//                        .build();
//
//                PestoFTCConfig.follower.followPath(pathChain);
//                while (opModeIsActive() && !isStopRequested() && gamepad1.y) {
//                    MotorCortex.update();
//                    PestoFTCConfig.follower.update();
//                }
//                PestoFTCConfig.follower.breakFollowing();
            }

            if (gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER)) {
                brakeSubsystem.setState(brakeSubsystem.getState() == BrakeSubsystem.BrakeState.DOWN ? BrakeSubsystem.BrakeState.UP : BrakeSubsystem.BrakeState.DOWN);
            }

            if (intaking) {
                state = RobotState.INTAKE;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
            }

            if (outtaking && state != RobotState.OUTTAKE) {
                state = RobotState.OUTTAKE;
//                xModePosition = PestoFTCConfig.follower.getPose();

                blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
            }

            if (rejecting) {
                state = RobotState.REJECT;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.REJECT);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
            }

            if (neutralizing) {
                state = RobotState.NEUTRAL;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
            }

            if (revving) {
                hoodSubsystem.setAngleDirect(PestoFTCConfig.interpolatorHood.getValue(cam_distance));
                outtakeSubsystem.setPowerDirect(PestoFTCConfig.interpolatorShooter.getValue(cam_distance));
            } else {
                outtakeSubsystem.setPowerDirect(0.0);
            }

            if (gamepad1.b)
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUTISH);
            else
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUT);

            blockerSubsystem.update();
            hoodSubsystem.update();
            intakeSubsystem.update();
            indexerSubsystem.update();
            turretSubsystem.update();
            brakeSubsystem.update();

            telemetry.addData("hood", PestoFTCConfig.interpolatorHood.getValue(cam_distance));
            telemetry.addData("shooter", PestoFTCConfig.interpolatorShooter.getValue(cam_distance));
            telemetry.update();
        }

        visionPortal.close();
    }
}
