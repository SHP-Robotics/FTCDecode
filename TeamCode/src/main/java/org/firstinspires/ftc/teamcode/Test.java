package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@TeleOp(name = "Test")
public class Test extends BaseRobot {
    @Override
    public void runOpMode() {
        MotorCortex.initialize(hardwareMap);

        CortexLinkedMotor frontLeft = MotorCortex.getMotor("frontLeft");
        CortexLinkedMotor frontRight = MotorCortex.getMotor("frontRight");
        CortexLinkedMotor backLeft = MotorCortex.getMotor("backLeft");
        CortexLinkedMotor backRight = MotorCortex.getMotor("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        frontLeft.setPowerResult(1.0);
        sleep(1000);
        frontLeft.setPowerResult(0.0);

        frontRight.setPowerResult(1.0);
        sleep(1000);
        frontRight.setPowerResult(0.0);

        backLeft.setPowerResult(1.0);
        sleep(1000);
        backLeft.setPowerResult(0.0);

        backRight.setPowerResult(1.0);
        sleep(1000);
        backRight.setPowerResult(0.0);
    }
}
