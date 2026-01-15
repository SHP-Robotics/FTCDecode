package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.BlueFarPaths.PathState.SECOND_PATH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;

@Autonomous(name = "Test")
public class Test extends BaseRobot {
    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        Utils.clear();
        super.initialize();

        BlueFarPaths.PathState state = SECOND_PATH;

        telemetry.addData("endpoint", state.getPath().getEndpoint());
        telemetry.addData("point 1", state.getPath().curves.get(0).getPose(1.0));
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
