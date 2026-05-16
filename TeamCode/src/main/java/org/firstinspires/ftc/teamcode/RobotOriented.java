package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@TeleOp(name = "Angela")
public class RobotOriented extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        boolean braking = false;
        boolean rotationLocked = false;
        double angle = 0.0;

        boolean usingOdometry = true;

        super.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            tracker.update();
            teleOpController.updateSpeed(gamepad1);

            boolean isStatic = tracker.getRobotVelocity().getMagnitude() < 1.0;
            mecanumController.setIsStatic(isStatic);

            if (gamepadInterface1.isKeyDown(GamepadKey.Y)) {
                usingOdometry = !usingOdometry;
                if (usingOdometry)
                    teleOpController.useTrackerIMU(tracker);
                else
                    teleOpController.useIMU();
            }

            if (gamepadInterface1.isKeyDown(GamepadKey.B)) {
                braking = !braking;
                mecanumController.setZeroPowerBehavior(braking ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
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

            if (gamepad1.x) {
                tracker.reset();
                teleOpController.resetIMU();
            }

            boolean intaking = gamepad1.right_trigger > 0.05;
            boolean outtaking = !intaking && gamepad1.left_trigger > 0.05;
            boolean rejecting = !intaking && !outtaking && gamepad1.a;
            boolean neutralizing = !intaking && !outtaking && !rejecting;

            if (rotationLocked) {
                double correction = angle - MathUtils.normalizeAngle(tracker.getCurrentPosition().getHeadingRadians(), angle);
                correction = correction * -0.6; // 0.6 == headingKP
                correction += Math.signum(correction) * PestoFTCConfig.STATIC_DRIVE;

                teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, correction);
            } else if (outtaking) {
                LLResult result = limelight.getLatestResult();

                List<LLResultTypes.FiducialResult> results = result.getFiducialResults();

                if (result != null && result.isValid() && results.size() > 0 && results.get(0).getFiducialId() != 21) {
                    double rotate = -result.getTx()*PestoFTCConfig.KP;
                    rotate = Math.min(1, Math.max(-1, rotate));

                    if (rotate < 0)
                        rotate -= PestoFTCConfig.STATIC_DRIVE;
                    else
                        rotate += PestoFTCConfig.STATIC_DRIVE;

                    teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
                    telemetry.addData("tx", result.getTx());
                } else {
                    telemetry.addLine("No Target :(");
                    teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
            } else
                teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (!outtaking && state == RobotState.OUTTAKE) {
                FrontalLobe.removeMacros("outtake");
            }

            if (intaking) {
                state = RobotState.INTAKE;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);
            }

            if (outtaking && state != RobotState.OUTTAKE) {
                state = RobotState.OUTTAKE;

                FrontalLobe.useMacro("outtake");
            }

            if (rejecting) {
                state = RobotState.REJECT;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.REJECT);
                feederSubsystem.setState(FeederSubsystem.FeederState.REVERSE);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);
            }

            if (neutralizing) {
                state = RobotState.NEUTRAL;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);
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
                    hoodSubsystem.setState(HoodSubsystem.HoodState.FAR);
                    outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_FAR);
                } else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.MID) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.CLOSE);
                    outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_CLOSE);
                } else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.FAR) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.MID);
                    outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_MIDDLE);
                }
            }

            feederSubsystem.update();
            hoodSubsystem.update();
            intakeSubsystem.update();
            outtakeSubsystem.update();
            indexerSubsystem.update();

            telemetry.addData("x", tracker.getCurrentPosition().getX());
            telemetry.addData("y", tracker.getCurrentPosition().getY());
            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.addData("teleop r", teleOpController.getHeading());
            telemetry.addData("using odometry", !usingOdometry);
            telemetry.addData("target", intakeSubsystem.dropdownTarget);
            telemetry.addData("pitch", intakeSubsystem.imu.getRobotYawPitchRollAngles().getPitch());
            telemetry.update();
        }
    }
}
