package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "Thanks Andrew")
public class Drive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        super.initialize();

        boolean rotationLocked = false;
        double angle = 0.0;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            tracker.update();

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

            if (gamepadInterface1.isKeyDown(GamepadKey.RIGHT_BUMPER)) {
                rotationLocked = !rotationLocked;

                double[] angles = new double[]{0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2};
                double bestDist = Double.POSITIVE_INFINITY;

                for (double x : angles) {
                    double dist = Math.abs(x - MathUtils.normalizeAngle(tracker.getCurrentPosition().getHeadingRadians(), x));

                    if (dist < bestDist) {
                        bestDist = dist;
                        angle = x;
                    }
                }
            }

            if (rotationLocked) {
                double correction = angle - MathUtils.normalizeAngle(tracker.getCurrentPosition().getHeadingRadians(), angle);
                correction = correction * -0.6; // 0.6 == headingKP
                correction += Math.signum(correction) * PestoFTCConfig.STATIC_DRIVE;

                teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, correction);
            } else {
                teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            boolean intaking = gamepad1.right_trigger > 0.05;
            boolean outtaking = !intaking && gamepad1.left_trigger > 0.05;
            boolean rejecting = !intaking && !outtaking && gamepad1.a;
            boolean neutralizing = !intaking && !outtaking && !rejecting;

            if (state == RobotState.OUTTAKE) {
                if (outtakeSubsystem.isBusy())
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                else
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
            }

            if (!outtaking && state == RobotState.OUTTAKE) {
                FrontalLobe.removeMacros("outtake");

                if (hoodSubsystem.getState() == HoodSubsystem.HoodState.CLOSE)
                    turretSubsystem.setState(TurretSubsystem.TurretState.STRAIGHT);
                else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.MID)
                    turretSubsystem.setState(TurretSubsystem.TurretState.STRAIGHT);
                else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.FAR)
                    turretSubsystem.setState(TurretSubsystem.TurretState.RIGHT);
            }

            if (gamepad1.dpad_left) {
                turretSubsystem.setState(TurretSubsystem.TurretState.MANUAL);
                while (opModeIsActive() && !isStopRequested() && gamepad1.dpad_left) {
                    turretSubsystem.setPower(0.3);
                }
                turretSubsystem.setPower(0.0);
            }

            if (gamepad1.dpad_right) {
                turretSubsystem.setState(TurretSubsystem.TurretState.MANUAL);
                while (opModeIsActive() && !isStopRequested() && gamepad1.dpad_right) {
                    turretSubsystem.setPower(-0.3);
                }
                turretSubsystem.setPower(0.0);
            }

            if (gamepad1.dpad_up && gamepad1.y) {
                turretSubsystem.rezero();
                turretSubsystem.setState(TurretSubsystem.TurretState.STRAIGHT);
            }

            if (gamepadInterface1.isKeyDown(GamepadKey.LEFT_BUMPER)) {
                brakeSubsystem.setState(brakeSubsystem.getState() == BrakeSubsystem.BrakeState.DOWN ? BrakeSubsystem.BrakeState.UP : BrakeSubsystem.BrakeState.DOWN);
            }

            if (intaking) {
                state = RobotState.INTAKE;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
            }

            if (outtaking && state != RobotState.OUTTAKE) {
                state = RobotState.OUTTAKE;

                FrontalLobe.useMacro("outtake");
            }

            if (rejecting) {
                state = RobotState.REJECT;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.REJECT);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
            }

            if (neutralizing) {
                state = RobotState.NEUTRAL;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
            }



//            // Nice little LED display on the gamepad
            if (hoodSubsystem.getState() == HoodSubsystem.HoodState.CLOSE)
                gamepad1.setLedColor(0, 255, 0, Integer.MAX_VALUE);

            if (hoodSubsystem.getState() == HoodSubsystem.HoodState.MID)
                gamepad1.setLedColor(0, 0, 255, Integer.MAX_VALUE);

            if (hoodSubsystem.getState() == HoodSubsystem.HoodState.FAR)
                gamepad1.setLedColor(255, 0, 0, Integer.MAX_VALUE);

//            // Cycle hood modes
            if (gamepadInterface1.isKeyDown(GamepadKey.TOUCHPAD)) {
                if (hoodSubsystem.getState() == HoodSubsystem.HoodState.CLOSE) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.MID);
                    outtakeSubsystem.setRPM(PestoFTCConfig.SHOOTER_MIDDLE);
                    outtakeSubsystem.setFFPower(PestoFTCConfig.SHOOTER_FF_MIDDLE);
                    turretSubsystem.setState(TurretSubsystem.TurretState.STRAIGHT);
                } else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.MID) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.FAR);
                    outtakeSubsystem.setRPM(PestoFTCConfig.SHOOTER_FAR);
                    outtakeSubsystem.setFFPower(PestoFTCConfig.SHOOTER_FF_FAR);
                    turretSubsystem.setState(TurretSubsystem.TurretState.RIGHT);
                } else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.FAR) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.CLOSE);
                    outtakeSubsystem.setRPM(PestoFTCConfig.SHOOTER_CLOSE);
                    outtakeSubsystem.setFFPower(PestoFTCConfig.SHOOTER_FF_CLOSE);
                    turretSubsystem.setState(TurretSubsystem.TurretState.STRAIGHT);
                }
            }

            if (gamepad1.b)
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUTISH);
            else
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUT);

            blockerSubsystem.update();
            hoodSubsystem.update();
            intakeSubsystem.update();
            outtakeSubsystem.update();
            indexerSubsystem.update();
            turretSubsystem.update();
            brakeSubsystem.update();

//            telemetry.addData("x", tracker.getCurrentPosition().getX());
//            telemetry.addData("y", tracker.getCurrentPosition().getY());
//            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.addData("turret", turretSubsystem.getPosition());
            telemetry.addData("shooter", outtakeSubsystem.getRPM());
            telemetry.addData("target", outtakeSubsystem.getTargetRPM());
            telemetry.update();
        }
    }
}
