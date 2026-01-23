package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@TeleOp(name = "Field Oriented")
public class FieldOriented extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = false;
        super.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            gamepadInterface1.update();
            teleOpController.updateSpeed(gamepad1);

            if (gamepad1.x) {
                teleOpController.resetIMU();
            }

            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}
