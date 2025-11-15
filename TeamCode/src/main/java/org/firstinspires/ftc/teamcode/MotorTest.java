package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.hardware.CortexLinkedCRServo;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@TeleOp(name = "Motor Test")
public class MotorTest extends BaseRobot {
    @Override
    public void runOpMode() {
        MotorCortex.initialize(hardwareMap);

        CortexLinkedCRServo feederLeft = MotorCortex.getCRServo("feederLeft");
        CortexLinkedCRServo feederRight = MotorCortex.getCRServo("feederRight");

        waitForStart();

        feederLeft.setPowerResult(0.3);
        sleep(1000);
        feederRight.setPowerResult(0.3);
        sleep(1000);
    }
}
