package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeOuttakeSubsystem;

@TeleOp(name = "CP?")
public class Drive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = false;

        super.initialize();

//        boolean rotationLocked = false;
//        double angle = 0.0;

        waitForStart();

        tracker.reset();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            tracker.update();

//            if (gamepad1.x) {
//                tracker.reset();
//                teleOpController.resetIMU();
//            }

//            if (gamepadInterface1.isKeyDown(GamepadKey.RIGHT_BUMPER)) {
//                rotationLocked = !rotationLocked;
//
//                double[] angles = new double[]{0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2};
//                double bestDist = Double.POSITIVE_INFINITY;
//
//                for (double x : angles) {
//                    double dist = Math.abs(x - MathUtils.normalizeAngle(tracker.getCurrentPosition().getHeadingRadians(), x));
//
//                    if (dist < bestDist) {
//                        bestDist = dist;
//                        angle = x;
//                    }
//                }
//            }

//            if (rotationLocked) {
//                double correction = angle - MathUtils.normalizeAngle(tracker.getCurrentPosition().getHeadingRadians(), angle);
//                correction = correction * -0.6; // 0.6 == headingKP
//                correction += Math.signum(correction) * PestoFTCConfig.STATIC_DRIVE;
//
//                teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, correction);
//            } else {
//                teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            }

            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            boolean intaking = gamepad1.right_trigger > 0.05;
            boolean outtaking = !intaking && gamepad1.left_trigger > 0.05;
            boolean rejecting = !intaking && !outtaking && gamepad1.a;
            boolean neutralizing = !intaking && !outtaking && !rejecting;


            if (!outtaking && state == RobotState.OUTTAKE) {
                FrontalLobe.removeMacros("outtake");
            }

            if (intaking) {
                state = RobotState.INTAKE;

                intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.INTAKE);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
            }

            if (outtaking && state != RobotState.OUTTAKE) {
                state = RobotState.OUTTAKE;

                FrontalLobe.useMacro("outtake");
            }

            if (rejecting) {
                state = RobotState.REJECT;

                intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.REJECT);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.NEUTRAL);
            }

            if (neutralizing) {
                state = RobotState.NEUTRAL;

                intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.NEUTRAL);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);
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

                } else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.MID) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.FAR);

                } else if (hoodSubsystem.getState() == HoodSubsystem.HoodState.FAR) {
                    hoodSubsystem.setState(HoodSubsystem.HoodState.CLOSE);
                }
            }


            blockerSubsystem.update();
            hoodSubsystem.update();
            intakeOuttakeSubsystem.update();


//            telemetry.addData("x", tracker.getCurrentPosition().getX());
//            telemetry.addData("y", tracker.getCurrentPosition().getY());
//            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
//            telemetry.addData("turret", turretSubsystem.getPosition());
//            telemetry.addData("shooter", outtakeSubsystem.getRPM());
//            telemetry.addData("target", outtakeSubsystem.getTargetRPM());
//            telemetry.update();
        }
    }
}
