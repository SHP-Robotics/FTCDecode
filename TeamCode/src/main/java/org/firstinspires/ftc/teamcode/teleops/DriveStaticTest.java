package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@TeleOp(name = "Drive Static Test", group = "Static Tests")
public class DriveStaticTest extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        super.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            tracker.update();

            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("Y", -gamepad1.left_stick_y);
            telemetry.addData("X", gamepad1.left_stick_x);
            telemetry.addData("R", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}