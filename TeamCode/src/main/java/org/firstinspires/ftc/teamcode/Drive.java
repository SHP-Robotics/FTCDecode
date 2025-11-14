package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@TeleOp(name = "Thanks Andrew")
public class Drive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        super.runOpMode();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();

            if (gamepad1.b) {
                tracker.reset();
                teleOpController.resetIMU();
            }

            teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            boolean intaking = gamepad1.right_trigger > 0.05;
            boolean outtaking = !intaking && gamepad1.a;
            boolean rejecting = !intaking && !outtaking && gamepad1.left_trigger > 0.05;
            boolean neutralizing = !intaking && !outtaking && !rejecting;

            if (intaking) {
                state = RobotState.INTAKE;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
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
            }

            if (neutralizing) {
                state = RobotState.NEUTRAL;

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
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
                if (hoodSubsystem.getState() == HoodSubsystem.HoodState.CLOSE)
                    hoodSubsystem.setState(HoodSubsystem.HoodState.MID);
                else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.MID)
                    hoodSubsystem.setState(HoodSubsystem.HoodState.FAR);
                else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.FAR)
                    hoodSubsystem.setState(HoodSubsystem.HoodState.CLOSE);
            }

            telemetry.addData("x", tracker.getCurrentPosition().getX());
            telemetry.addData("y", tracker.getCurrentPosition().getY());
            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
}
