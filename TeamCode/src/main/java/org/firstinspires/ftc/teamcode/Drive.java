package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "Thanks Andrew")
public class Drive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        super.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
//            tracker.update();

            if (gamepad1.x) {
//                tracker.reset();
                teleOpController.resetIMU();
            }

            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            boolean intaking = gamepad1.right_trigger > 0.05;
            boolean outtaking = !intaking && gamepad1.left_trigger > 0.05;
            boolean rejecting = !intaking && !outtaking && gamepad1.a;
            boolean neutralizing = !intaking && !outtaking && !rejecting;

            // TOUCHPAD - SHOOTER, HOOD, TURRET
            // GREEN - CLOSE, BLUE - MIDDLE, RED - FAR
            // GREEN and BLUE are straight forward, RED is aligned

            // DPAD DOWN - BRAKE TOGGLE, DEFAULT FLOAT

//            if (!outtaking && state == RobotState.OUTTAKE) {
//                FrontalLobe.removeMacros("outtake");
//            }

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
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
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
                    outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_MIDDLE);
                    turretSubsystem.setState(TurretSubsystem.TurretState.STRAIGHT);
                } else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.MID) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.FAR);
                    outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_FAR);
                    turretSubsystem.setState(TurretSubsystem.TurretState.LEFT);
                } else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.FAR) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.CLOSE);
                    outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_CLOSE);
                    turretSubsystem.setState(TurretSubsystem.TurretState.STRAIGHT);
                }
            }

            blockerSubsystem.update();
            hoodSubsystem.update();
            intakeSubsystem.update();
            outtakeSubsystem.update();
            indexerSubsystem.update();
            turretSubsystem.update();

//            telemetry.addData("x", tracker.getCurrentPosition().getX());
//            telemetry.addData("y", tracker.getCurrentPosition().getY());
//            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
//            telemetry.update();
        }
    }
}
