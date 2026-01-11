package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Turret Static Test", group = "Static Tests")
public class TurretStaticTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        CortexLinkedMotor turret = MotorCortex.getMotor("turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

            turret.setPowerResult(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("power", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.update();
        }
    }
}
