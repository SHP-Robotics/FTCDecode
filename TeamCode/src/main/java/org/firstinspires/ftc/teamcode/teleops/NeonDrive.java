package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@TeleOp(name = "Neon Drive")
public class NeonDrive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        super.initialize();

        DcMotor encoder = MotorCortex.getMotor(3, 173);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

//            dogGear.setPosition(0.75);
//
//            mecanumController.frontLeft.motor.setPower(-1.0);
//            mecanumController.frontRight.motor.setPower(-1.0);
//            mecanumController.backLeft.motor.setPower(0.3);
//            mecanumController.backRight.motor.setPower(0.3);

            telemetry.addData("encoder", encoder.getCurrentPosition());
            telemetry.update();
        }

        turretSubsystem.visionPortal.close();
    }
}
