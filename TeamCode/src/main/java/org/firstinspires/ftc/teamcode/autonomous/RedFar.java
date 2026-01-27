package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.DONE;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.EIGHTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.FIFTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.FIRST_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.FOURTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.NINTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.SECOND_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.SEVENTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.SIXTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.RedFarPaths.PathState.THIRD_PATH;

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

@Autonomous(name = "Red Far")
public class RedFar extends BaseRobot {
    RedFarPaths.PathState state;
    PathFollower pathFollower;
    double start;

    public void nextState() {
        switch (state) {
            case FIRST_PATH:
                mecanumController.drive(0, 0, 0);
                FrontalLobe.useMacro("outtake");
                while (Utils.timer(8.5, "outtake") && opModeIsActive() && !isStopRequested()) {
                    FrontalLobe.update();
                    MotorCortex.update();
                    tracker.update();
                    pathFollower.update();

                    double distanceToEndpoint = Pose.dist(tracker.getCurrentPosition(), pathFollower.getPathContainer().getEndpoint());
                    mecanumController.setIsStatic(distanceToEndpoint > 0.3 && tracker.getRobotVelocity().getMagnitude() < 0.5);

                    feederSubsystem.update();
                    hoodSubsystem.update();
                    indexerSubsystem.update();
                    intakeSubsystem.update();
                    outtakeSubsystem.update();
                }

                feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);

                state = SECOND_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = RedFarPaths.getPathFollower(state);
                break;
            case SECOND_PATH:
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);

                state = THIRD_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = RedFarPaths.getPathFollower(state, 0.2);
                break;
            case THIRD_PATH:
                feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);

                state = FOURTH_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = RedFarPaths.getPathFollower(state);
                break;
            case FOURTH_PATH:
                state = FIFTH_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = RedFarPaths.getPathFollower(state);
                break;
            case FIFTH_PATH:
                state = SIXTH_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = RedFarPaths.getPathFollower(state);
                break;
            case SIXTH_PATH:
                FrontalLobe.removeMacros("");
                Utils.clear();

                FrontalLobe.useMacro("outtake");
                while (Utils.timer(8.5, "outtake") && opModeIsActive() && !isStopRequested()) {
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

                state = SEVENTH_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = RedFarPaths.getPathFollower(state);
                break;
            case SEVENTH_PATH:
                state = EIGHTH_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = RedFarPaths.getPathFollower(state);

                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);

                break;
            case EIGHTH_PATH:
                state = NINTH_PATH;
                start = System.nanoTime() / 1E9;
                pathFollower = RedFarPaths.getPathFollower(state);

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);

                break;
            case NINTH_PATH:
                FrontalLobe.removeMacros("");
                Utils.clear();

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
        pathFollower = RedFarPaths.getPathFollower(state);

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
                telemetry.addData("tracker", tracker.getCurrentPosition());
                telemetry.update();
                continue;
            }

            if (pathFollower == null) {
                continue;
            }

            pathFollower.update();

            telemetry.addData("target", pathFollower.getPathContainer().getCurrentPosition());
            telemetry.update();

            if (pathFollower.isFinished() || (currentTime - start) > state.getTimer())
                nextState();
        }
    }
}
