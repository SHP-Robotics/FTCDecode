package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;

@Autonomous(name = "Red One Two Three")
public class RedOneTwoThree extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        Utils.clear();
        super.initialize();

        hoodSubsystem.setState(HoodSubsystem.HoodState.AUTO_FAR);
        outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_FAR);

        waitForStart();

        FrontalLobe.useMacro("auto - outtake");
        while (Utils.timer(8.5, "auto - outtake") && opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();

            feederSubsystem.update();


            hoodSubsystem.update();
            indexerSubsystem.update();
            intakeSubsystem.update();
            outtakeSubsystem.update();
        }

        mecanumController.drive(0, 0.8, 0);
        sleep(2000);
        mecanumController.drive(0, 0, 0);
    }
}
