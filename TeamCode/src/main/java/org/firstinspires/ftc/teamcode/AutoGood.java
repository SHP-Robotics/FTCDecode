package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@Autonomous(name = "AutoGood")
public class AutoGood extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;

        super.runOpMode();

        waitForStart();

        teleOpController.driveRobotCentric(-0.6, 0, 0);
        sleep(300);
        teleOpController.driveRobotCentric(0, 0, 0);
    }
}
