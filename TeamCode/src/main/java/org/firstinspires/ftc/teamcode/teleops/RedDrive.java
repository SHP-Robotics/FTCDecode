package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Red Drive")
public class RedDrive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        boolean revving = false;
        boolean outtaking;

        super.initialize();

        turretSubsystem.setAcceptedTags(PestoFTCConfig.RED_TAGS);
        turretSubsystem.setState(TurretSubsystem.TurretState.STRAIGHT);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        double cam_distance = 0.0;

        waitForStart();

        turretSubsystem.rezero();

        while (opModeIsActive() && !isStopRequested()) {
            // only for tuning purposes
            PestoFTCConfig.recalculate_interpolators();

            List<AprilTagDetection> currentDetections = turretSubsystem.aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            tracker.update();

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

            cam_distance = turretSubsystem.getAprilTagDistance();

            telemetry.addData("distance", cam_distance);
            telemetry.addData("hood", PestoFTCConfig.interpolatorHood.getValue(cam_distance));
            telemetry.addData("shooter", PestoFTCConfig.interpolatorShooter.getValue(cam_distance));
            telemetry.update();
        }

        turretSubsystem.visionPortal.close();
    }
}
