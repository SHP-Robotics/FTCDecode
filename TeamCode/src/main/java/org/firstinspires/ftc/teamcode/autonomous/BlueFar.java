package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.BlueFarPaths.PathState.DONE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@Autonomous(name = "Blue Far")
public class BlueFar extends BaseRobot {
    BlueFarPaths.PathState state;
    PathFollower pathFollower;
    double start;

    public void nextState() {
        switch (state) {
            case FIRST_PATH:
                mecanumController.drive(0, 0, 0);
                FrontalLobe.useMacro("outtake");
                while (Utils.timer(7.0, "outtake")) {
                    FrontalLobe.update();
                    MotorCortex.update();

                    feederSubsystem.update();
                    hoodSubsystem.update();
                    indexerSubsystem.update();
                    intakeSubsystem.update();
                    outtakeSubsystem.update();
                }

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);

                state = DONE;
                break;
            case DONE:
                break;
        }
    }

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        Utils.clear();
        super.initialize();

        state = BlueFarPaths.PathState.FIRST_PATH;
        pathFollower = BlueFarPaths.getPathFollower(state);

        hoodSubsystem.setState(HoodSubsystem.HoodState.AUTO_FAR);
        outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_FAR);

        waitForStart();

        start = System.nanoTime() / 1E9;

        while (opModeIsActive() && !isStopRequested()) {
            double currentTime = System.nanoTime() / 1E9;

            FrontalLobe.update();
            MotorCortex.update();
            tracker.update();

            boolean isStatic = tracker.getRobotVelocity().getMagnitude() < 1.0;
            mecanumController.setIsStatic(isStatic);

            feederSubsystem.update();
            hoodSubsystem.update();
            indexerSubsystem.update();
            intakeSubsystem.update();
            outtakeSubsystem.update();

            if (state == DONE) {
                FrontalLobe.driveController.drive(0, 0, 0);
                continue;
            }

            if (pathFollower.isFinished(0.2, 0.05) || (currentTime - start) > state.getTimer())
                nextState();

            if (pathFollower != null)
                pathFollower.update();

            telemetry.update();
        }
    }
}
