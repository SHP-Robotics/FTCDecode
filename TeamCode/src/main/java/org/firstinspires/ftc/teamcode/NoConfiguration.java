package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "No Configuration")
public class NoConfiguration extends LinearOpMode {
    @Override
    public void runOpMode() {
        MotorCortex.initialize(hardwareMap);

        DcMotor port0 = MotorCortex.getMotor(0, true);
        DcMotor port1 = MotorCortex.getMotor(1, true);
        DcMotor port2 = MotorCortex.getMotor(2, true);
        DcMotor port3 = MotorCortex.getMotor(3, true);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

            telemetry.addData("port - 0", port0.getCurrentPosition());
            telemetry.addData("port - 1", port1.getCurrentPosition());
            telemetry.addData("port - 2", port2.getCurrentPosition());
            telemetry.addData("port - 3", port3.getCurrentPosition());
            telemetry.update();
        }
    }
}
