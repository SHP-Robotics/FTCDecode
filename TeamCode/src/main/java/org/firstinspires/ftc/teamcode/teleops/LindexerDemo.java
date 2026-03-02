package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@TeleOp(name = "Lindexer Demo")
public class LindexerDemo extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        super.initialize();

        telemetry.addLine("Use gamepad1.left_stick_y to control the lindexer");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();
            
            indexerSubsystem.setPositionDirect(-gamepad1.left_stick_y);

            telemetry.addLine("Use gamepad1.left_stick_y to control the lindexer");
            telemetry.addData("position", Math.max(-gamepad1.left_stick_y, 0));
            telemetry.update();
        }

        turretSubsystem.visionPortal.close();
    }
}
