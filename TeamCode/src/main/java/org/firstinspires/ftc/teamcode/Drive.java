package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@TeleOp(name = "Drive")
public class Drive extends BaseRobot {
    public enum DistanceMode {
        CLOSE,
        MID,
        FAR
    }

    private double power_close = 0.47;
    private double power_mid = 0.55;
    private double power_far = 0.65;

    DistanceMode mode;

    @Override
    public void runOpMode() {
        // intake on R1
        // outtake on R2
        // reject on L2

        PestoFTCConfig.initializePinpoint = true;

        super.runOpMode();

        mode = DistanceMode.MID;
        outtakeSubsystem.setPower(power_mid);
        gamepad1.setLedColor(255, 255, 0, Integer.MAX_VALUE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            teleOpController.updateSpeed(gamepad1);
//            tracker.update();
//
//            if (gamepad1.b) {
//                tracker.reset();
//                teleOpController.resetIMU();
//            }

            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            boolean intaking = gamepad1.right_bumper;
            boolean outtaking = !intaking && gamepad1.right_trigger > 0.05;
            boolean rejecting = !intaking && !outtaking && gamepad1.left_bumper;
            boolean neutralizing = !intaking && !outtaking && !rejecting;

            if (intaking) {
                state = RobotState.INTAKE;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                feederSubsystem.setState(FeederSubsystem.FeederState.INTAKE);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
            }

            if (outtaking && state != RobotState.OUTTAKE) {
                state = RobotState.OUTTAKE;

                FrontalLobe.useMacro("outtake");
            }

            if (rejecting) {
                state = RobotState.REJECT;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.REJECT);
                feederSubsystem.setState(FeederSubsystem.FeederState.REJECT);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
            }

            if (neutralizing) {
                state = RobotState.NEUTRAL;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                feederSubsystem.setState(FeederSubsystem.FeederState.NEUTRAL);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
            }

            if (gamepadInterface1.isKeyDown(GamepadKey.TOUCHPAD)) {
                if (mode == DistanceMode.CLOSE) {
                    mode = DistanceMode.MID;
                    gamepad1.setLedColor(255, 255, 0, Integer.MAX_VALUE);
                    outtakeSubsystem.setPower(power_mid);
                } else if (mode == DistanceMode.MID) {
                    mode = DistanceMode.FAR;
                    gamepad1.setLedColor(0, 255, 0, Integer.MAX_VALUE);
                    outtakeSubsystem.setPower(power_far);
                } else if (mode == DistanceMode.FAR) {
                    mode = DistanceMode.CLOSE;
                    gamepad1.setLedColor(255, 0, 0, Integer.MAX_VALUE);
                    outtakeSubsystem.setPower(power_close);
                }
            }

            intakeSubsystem.update();
            feederSubsystem.update();
            outtakeSubsystem.update();

//            telemetry.addData("x", tracker.getCurrentPosition().getX());
//            telemetry.addData("y", tracker.getCurrentPosition().getY());
//            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.addData("touchpad", gamepadInterface1.isKey(GamepadKey.TOUCHPAD));
            telemetry.update();
        }
    }
}
