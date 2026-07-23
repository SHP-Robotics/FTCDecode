package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Blue Drive")
public class BlueDrive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        boolean revving = false;
        boolean outtaking;

        super.initialize();

        DcMotor encoder = MotorCortex.getMotor(3, 173);

        turretSubsystem.setAcceptedTags(PestoFTCConfig.BLUE_TAGS);
        turretSubsystem.setTargetPosition(0);
        turretSubsystem.setVisionOffset(0);
        turretSubsystem.setDetectAprilTag(true);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        double cam_distance = 0.0;

        waitForStart();

        int encoderStart = encoder.getCurrentPosition();

//        turretSubsystem.rezero();

        long start;
        while (opModeIsActive() && !isStopRequested() && !(gamepad1.left_bumper && gamepad1.right_bumper)) {
            start = System.nanoTime();

            PestoFTCConfig.recalculate_interpolators();

            List<AprilTagDetection> currentDetections = turretSubsystem.aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            tracker.update();

            if (gamepadInterface1.isKeyUp(GamepadKey.DPAD_DOWN))
                turretSubsystem.setTargetPosition(turretSubsystem.getTargetPosition() == 0 ? 1550 : 0);

            if (gamepad1.x) {
                tracker.reset();
                teleOpController.resetIMU();
            }

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            boolean intaking = gamepad1.right_trigger > 0.05;
            outtaking = !intaking && gamepadInterface1.getTimeDown(GamepadKey.LEFT_TRIGGER) > 0.3;
            revving = (revving != gamepadInterface1.isClicked(GamepadKey.LEFT_TRIGGER, 0.3)) || outtaking;
            boolean rejecting = !intaking && !outtaking && gamepad1.a;
            boolean neutralizing = !intaking && !outtaking && !rejecting;

            if (state == RobotState.OUTTAKE) {
                if (!outtakeSubsystem.isBusy(PestoFTCConfig.interpolatorShooter.getValue(cam_distance)))
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                else
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
            }

            if (gamepad1.dpad_left) {
                while (opModeIsActive() && !isStopRequested() && gamepad1.dpad_left) {
                    MotorCortex.update();
                    teleOpController.driveFieldCentric(0, 0, 0);
                    turretSubsystem.setPower(0.3);
                }
                turretSubsystem.setPower(0.0);
                turretSubsystem.rezero();
                turretSubsystem.setTargetPosition(turretSubsystem.getTargetPosition());
            }

            if (gamepad1.dpad_right) {
                while (opModeIsActive() && !isStopRequested() && gamepad1.dpad_right) {
                    MotorCortex.update();
                    teleOpController.driveFieldCentric(0, 0, 0);
                    turretSubsystem.setPower(-0.3);
                }
                turretSubsystem.setPower(0.0);
                turretSubsystem.rezero();
                turretSubsystem.setTargetPosition(turretSubsystem.getTargetPosition());
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

            if (gamepad1.b)
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUTISH);
            else
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUT);

            if (revving) {
                hoodSubsystem.setAngleDirect(PestoFTCConfig.interpolatorHood.getValue(cam_distance));
                outtakeSubsystem.setPower(PestoFTCConfig.interpolatorShooter.getValue(cam_distance));
            } else {
                outtakeSubsystem.setPower(0.0);
            }

            dogGear.setPosition(0.00);
            blockerSubsystem.update();
            hoodSubsystem.update();
            intakeSubsystem.update();
            indexerSubsystem.update();
            turretSubsystem.update();
            outtakeSubsystem.update();

            cam_distance = turretSubsystem.lastAprilTag == null ? 0 : turretSubsystem.lastAprilTag.y;

            telemetry.addData("hz", 1E9 / (System.nanoTime() - start));
            telemetry.addData("dist", cam_distance);
            telemetry.addData("vel", outtakeSubsystem.getVelocity());
            telemetry.addData("exp", outtakeSubsystem.getExpectedVelocity(PestoFTCConfig.interpolatorShooter.getValue(cam_distance)));
            telemetry.addData("busy", outtakeSubsystem.isBusy(PestoFTCConfig.interpolatorShooter.getValue(cam_distance)));
            telemetry.update();

            dashboardTelemetry.addData("expected velocity", outtakeSubsystem.getExpectedVelocity(-PestoFTCConfig.interpolatorShooter.getValue(cam_distance)));
            dashboardTelemetry.addData("velocity", outtakeSubsystem.getVelocity());
            dashboardTelemetry.update();
        }

        if (isStopRequested())
            return;

        dogGear.setPosition(0.75);

        mecanumController.frontLeft.motor.setPower(0.0);
        mecanumController.frontRight.motor.setPower(0.0);
        mecanumController.backLeft.motor.setPower(0.0);
        mecanumController.backRight.motor.setPower(0.0);


        blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
        blockerSubsystem.update();

        intakeSubsystem.setPowerDirect(0.0);

        indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUT);
        indexerSubsystem.update();

        turretSubsystem.setPower(0.0);

        mecanumController.frontLeft.motor.setPower(0.2);
        mecanumController.frontRight.motor.setPower(0.2);

        sleep(750);

        mecanumController.frontLeft.motor.setPower(1.0);
        mecanumController.frontRight.motor.setPower(1.0);
        mecanumController.backLeft.motor.setPower(-1.0);
        mecanumController.backRight.motor.setPower(-1.0);

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

            if (encoder.getCurrentPosition() - encoderStart > 38000) {
                mecanumController.frontLeft.motor.setPower(0.0);
                mecanumController.frontRight.motor.setPower(0.0);
                mecanumController.backLeft.motor.setPower(0.0);
                mecanumController.backRight.motor.setPower(0.0);
            }

            telemetry.addData("encoder", encoder.getCurrentPosition());
            telemetry.update();
        }

        turretSubsystem.visionPortal.close();
    }
}
