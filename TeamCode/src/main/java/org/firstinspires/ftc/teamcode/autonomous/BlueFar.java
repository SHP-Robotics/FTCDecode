package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.BlueFarPaths.PathState.DONE;
import static org.firstinspires.ftc.teamcode.autonomous.BlueFarPaths.PathState.FIFTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.BlueFarPaths.PathState.FIRST_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.BlueFarPaths.PathState.FOURTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.BlueFarPaths.PathState.SECOND_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.BlueFarPaths.PathState.THIRD_PATH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
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
                FrontalLobe.useMacro("outtake");
                while (Utils.timer(7, "outtake") && opModeIsActive() && !isStopRequested()) {
                    FrontalLobe.update();
                    MotorCortex.update();
                    tracker.update();
                    pathFollower.update();

                    mecanumController.setIsStatic(tracker.getRobotVelocity().getMagnitude() < 1.0);

                    feederSubsystem.update();
                    hoodSubsystem.update();
                    indexerSubsystem.update();
                    intakeSubsystem.update();
                    outtakeSubsystem.update();
                }

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);

                state = SECOND_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = BlueFarPaths.getPathFollower(state);
                break;
            case SECOND_PATH:
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);

                state = THIRD_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = BlueFarPaths.getPathFollower(state, 0.2);
                break;
            case THIRD_PATH:
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);

                state = FOURTH_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = BlueFarPaths.getPathFollower(state);
                break;
            case FOURTH_PATH:
                state = FIFTH_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = BlueFarPaths.getPathFollower(state);
                break;
            case FIFTH_PATH:
                FrontalLobe.useMacro("outtake");
                while (Utils.timer(7, "outtake") && opModeIsActive() && !isStopRequested()) {
                    FrontalLobe.update();
                    MotorCortex.update();
                    tracker.update();
                    pathFollower.update();

                    mecanumController.setIsStatic(tracker.getRobotVelocity().getMagnitude() < 1.0);

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
                start = System.nanoTime() / 1E9;
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

        mecanumController.setStaticPower(PestoFTCConfig.DRIVE_STATIC);

        state = FIRST_PATH;
        pathFollower = BlueFarPaths.getPathFollower(state);

        hoodSubsystem.setState(HoodSubsystem.HoodState.CLOSE);
        outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_CLOSE);

        waitForStart();

        start = System.nanoTime() / 1E9;

        while (opModeIsActive() && !isStopRequested()) {
            double currentTime = System.nanoTime() / 1E9;

            FrontalLobe.update();
            MotorCortex.update();
            tracker.update();

            double distanceToEndpoint = Pose.dist(tracker.getCurrentPosition(), pathFollower.getPathContainer().getEndpoint());
            mecanumController.setIsStatic(distanceToEndpoint > 0.3 && tracker.getRobotVelocity().getMagnitude() < 0.5);

            feederSubsystem.update();
            hoodSubsystem.update();
            indexerSubsystem.update();
            intakeSubsystem.update();
            outtakeSubsystem.update();

            if (state == DONE) {
                FrontalLobe.driveController.drive(0, 0, 0);
                continue;
            }

            if (pathFollower == null)
                continue;

            pathFollower.update();

            telemetry.addData("target", pathFollower.getPathContainer().getCurrentPosition());
            telemetry.update();

            if (pathFollower.isFinished() || (currentTime - start) > state.getTimer())
                nextState();
        }
    }
}
