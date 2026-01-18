package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.DONE;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.EIGHTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.FIFTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.FOURTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.NINETH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.SECOND_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.SEVENTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.SIXTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.TENTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsRedClose.PathState.THIRD_PATH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@Autonomous(name = "close Auto red")
public class TestAutoRedClose extends BaseRobot {
    TestAutoPathsRedClose.PathState state;
    PathFollower pathFollower;
    double start;

    public void nextState() {
        switch (state) {
            case FIRST_PATH:
                turretSubsystem.setPosition(465.67);//increase for more to the left- stays the same entire auto

                while (Utils.timer(1.0, "turret") && opModeIsActive() && !isStopRequested()) {
                    FrontalLobe.update();
                    MotorCortex.update();

                    turretSubsystem.update();
                    tracker.update();
                    pathFollower.update();

                    telemetry.addData("time", (System.nanoTime() / 1E9) - Utils.getTime("turret"));
                    telemetry.update();
                }

                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.OUTTAKE);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
                while (Utils.timer(3.0, "outtake") && opModeIsActive() && !isStopRequested()) {
                    FrontalLobe.update();
                    MotorCortex.update();

                    tracker.update();
                    pathFollower.update();

                    blockerSubsystem.update();
                    hoodSubsystem.update();
                    intakeSubsystem.update();
                    outtakeSubsystem.update();
                    indexerSubsystem.update();
                    turretSubsystem.update();

                    if (outtakeSubsystem.isBusy())
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                    else
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                }

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);

                state = SECOND_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                Utils.clear();
                break;
            case SECOND_PATH:
                state = THIRD_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                break;
            case THIRD_PATH:
                state = FOURTH_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                break;
            case FOURTH_PATH:
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.OUTTAKE);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
                while (Utils.timer(3.0, "outtake") && opModeIsActive() && !isStopRequested()) {
                    FrontalLobe.update();
                    MotorCortex.update();

                    tracker.update();
                    pathFollower.update();

                    blockerSubsystem.update();
                    hoodSubsystem.update();
                    intakeSubsystem.update();
                    outtakeSubsystem.update();
                    indexerSubsystem.update();
                    turretSubsystem.update();

                    if (outtakeSubsystem.isBusy())
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                    else
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                }

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);

                state = FIFTH_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                Utils.clear();
                break;

            case FIFTH_PATH:
                state = SIXTH_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                break;

            case SIXTH_PATH:
                state = SEVENTH_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                break;

            case SEVENTH_PATH:
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.OUTTAKE);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
                while (Utils.timer(3.0, "outtake") && opModeIsActive() && !isStopRequested()) {
                    FrontalLobe.update();
                    MotorCortex.update();

                    tracker.update();
                    pathFollower.update();

                    blockerSubsystem.update();
                    hoodSubsystem.update();
                    intakeSubsystem.update();
                    outtakeSubsystem.update();
                    indexerSubsystem.update();
                    turretSubsystem.update();

                    if (outtakeSubsystem.isBusy())
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                    else
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                }

                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);

                state = EIGHTH_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);

                break;

            case EIGHTH_PATH:
                state = NINETH_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                break;

            case NINETH_PATH:
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.OUTTAKE);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
                while (Utils.timer(3.0, "outtake") && opModeIsActive() && !isStopRequested()) {
                    FrontalLobe.update();
                    MotorCortex.update();

                    tracker.update();
                    pathFollower.update();

                    blockerSubsystem.update();
                    hoodSubsystem.update();
                    intakeSubsystem.update();
                    outtakeSubsystem.update();
                    indexerSubsystem.update();
                    turretSubsystem.update();

                    if (outtakeSubsystem.isBusy())
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                    else
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);

                }


                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                blockerSubsystem.setState(BlockerSubsystem.BlockerState.BLOCK);

                state = TENTH_PATH;
                pathFollower = TestAutoPathsRedClose.getPathFollower(state);
                break;

            case TENTH_PATH:
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                state = DONE;
                pathFollower = null;
                Utils.clear();
                break;

        }





        start = System.nanoTime() / 1E9;
    }

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        Utils.clear();
        super.initialize();

        state = TestAutoPathsRedClose.PathState.FIRST_PATH;
        pathFollower = TestAutoPathsRedClose.getPathFollower(state);

        hoodSubsystem.setState(HoodSubsystem.HoodState.FAR);
        outtakeSubsystem.setRPM(PestoFTCConfig.SHOOTER_AUTO);
        outtakeSubsystem.setFFPower(PestoFTCConfig.SHOOTER_FF_AUTO);

        waitForStart();

        turretSubsystem.rezero();

        start = System.nanoTime() / 1E9;

        while (opModeIsActive() && !isStopRequested()) {
            double currentTime = System.nanoTime() / 1E9;

            FrontalLobe.update();
            MotorCortex.update();

            gamepadInterface1.update();
            tracker.update();

            blockerSubsystem.update();
            hoodSubsystem.update();
            intakeSubsystem.update();
            outtakeSubsystem.update();
            indexerSubsystem.update();
            turretSubsystem.update();
            brakeSubsystem.update();

            if (state == DONE) {
                FrontalLobe.driveController.drive(0, 0, 0);
                continue;
            }

            if (pathFollower.isFinished() || (currentTime - start) > state.getTimer())
                nextState();

            if (pathFollower != null)
                pathFollower.update();

            telemetry.addData("R", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.addData("target pose", pathFollower.getPathContainer().getCurrentPosition());
            telemetry.addData("shooter", outtakeSubsystem.getRPM());
            telemetry.addData("target", outtakeSubsystem.getTargetRPM());
            telemetry.update();
        }
    }
}
