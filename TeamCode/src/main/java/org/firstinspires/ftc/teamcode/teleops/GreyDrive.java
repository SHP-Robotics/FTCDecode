package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@Disabled
@TeleOp(name = "Grey Drive")
public class GreyDrive extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        super.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            dogGear.setPosition(0.75);

            mecanumController.frontLeft.motor.setPower(0.3);
            mecanumController.frontRight.motor.setPower(0.3);
            mecanumController.backLeft.motor.setPower(-0.3);
            mecanumController.backRight.motor.setPower(-0.3);
        }

        turretSubsystem.visionPortal.close();
    }
}
