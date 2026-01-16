package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.drivebases.trackers.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.processing.MotorCortex;

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

        waitForStart();

        double dL = 0.0;
        double dC = 0.0;
        double dR = 0.0;

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();

            if (gamepad1.b) {
                dL = 0.0;
                dC = 0.0;
                dR = 0.0;
            }

            dL += ((ThreeWheelOdometryTracker) tracker).leftOdometry.getInchesTravelled();
            dC += ((ThreeWheelOdometryTracker) tracker).centerOdometry.getInchesTravelled();
            dR += ((ThreeWheelOdometryTracker) tracker).rightOdometry.getInchesTravelled();

            telemetry.addData("dL", dL);
            telemetry.addData("dC", dC);
            telemetry.addData("dR", dR);
            telemetry.update();
        }
    }
}
