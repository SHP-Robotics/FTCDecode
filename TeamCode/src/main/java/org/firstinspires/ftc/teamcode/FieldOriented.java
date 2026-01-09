package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.variable.QualitativeData;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@TeleOp(name = "Field Oriented")
public class FieldOriented extends BaseRobot {
    PestoTelemetry pestoTelemetry;

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        PestoFTCConfig.initializeDrive = true;
        pestoTelemetry = FrontalLobe.pestoTelemetry;

        super.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            tracker.update();
            pestoTelemetry.clear();
            teleOpController.updateSpeed(gamepad1);

            if (gamepad1.b) {
                tracker.reset();
                teleOpController.resetIMU();
            }

            boolean intaking = gamepad1.right_trigger > 0.05 && -gamepad1.left_stick_y >= -0.1;
            boolean outtaking = !intaking && gamepad1.left_trigger > 0.05;
            boolean rejecting = !intaking && !outtaking && gamepad1.right_bumper;
            boolean neutralizing = !intaking && !outtaking && !rejecting;

            if (outtaking) {
                LLResult result = limelight.getLatestResult();

                List<LLResultTypes.FiducialResult> results = result.getFiducialResults();

                if (result != null && result.isValid() && results.size() > 0 && results.get(0).getFiducialId() != 21) {
                    double rotate = -result.getTx()*PestoFTCConfig.KP;
                    rotate = Math.min(1, Math.max(-1, rotate));

                    if (rotate < 0)
                        rotate -= PestoFTCConfig.STATIC_DRIVE;
                    else
                        rotate += PestoFTCConfig.STATIC_DRIVE;

                    teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
                    telemetry.addData("tx", result.getTx());
                } else {
                    telemetry.addLine("No Target :(");
                    teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
            } else
                teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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


            // Nice little LED display on the gamepad
            if (hoodSubsystem.getState() == HoodSubsystem.HoodState.CLOSE)
                gamepad1.setLedColor(0, 255, 0, Integer.MAX_VALUE);

            if (hoodSubsystem.getState() == HoodSubsystem.HoodState.MID)
                gamepad1.setLedColor(0, 0, 255, Integer.MAX_VALUE);

            if (hoodSubsystem.getState() == HoodSubsystem.HoodState.FAR)
                gamepad1.setLedColor(255, 0, 0, Integer.MAX_VALUE);

            // Cycle hood modes
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

            double x = Math.round(tracker.getCurrentPosition().getX() * 100) / 100.0;
            double y = Math.round(tracker.getCurrentPosition().getY() * 100) / 100.0;
            double r = Math.round(tracker.getCurrentPosition().getHeadingRadians() * 100) / 100.0;

            pestoTelemetry.addToDash(new QualitativeData("x, y, r", String.format("%.2f, %.2f, %.2f", x, y, r)));
            pestoTelemetry.addToDash(new QualitativeData("d", String.format("%.2f", hoodSubsystem.getDistance())));
            pestoTelemetry.update();

            telemetry.addData("x", tracker.getCurrentPosition().getX());
            telemetry.addData("y", tracker.getCurrentPosition().getY());
            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.addData("target", intakeSubsystem.dropdownTarget);
            telemetry.addData("pitch", intakeSubsystem.imu.getRobotYawPitchRollAngles().getPitch());
            telemetry.addData("dx", intakeSubsystem.imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
            telemetry.addData("dy", intakeSubsystem.imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate);
            telemetry.addData("dz", intakeSubsystem.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
//            telemetry.update();
            telemetry.update();
        }
    }
}
