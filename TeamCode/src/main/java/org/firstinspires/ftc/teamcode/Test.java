package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "Test")
public class Test extends BaseRobot {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        intakeSubsystem = new IntakeSubsystem();

        waitForStart();

        intakeSubsystem.setState(IntakeSubsystem.IntakeState.REJECT);

        long start = System.nanoTime();
        while (opModeIsActive() && !isStopRequested()) {
            intakeSubsystem.update();
            MotorCortex.update();

            telemetry.addData("pitch", intakeSubsystem.imu.getRobotYawPitchRollAngles().getPitch());
            telemetry.update();
        }
    }
}
