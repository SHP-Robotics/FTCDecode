package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.DONE;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.EIGHTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.FIFTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.FOURTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.SECOND_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.SEVENTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.SIXTH_PATH;
import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.THIRD_PATH;

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

@Autonomous(name = "Test Auto blue")
public class TestAutoBlue extends BaseRobot {
    TestAutoPathsBlue.PathState state;
    PathFollower pathFollower;
    double start;

    public void nextState() {
        switch (state) {
            case FIRST_PATH:
                turretSubsystem.setPosition(-445);//increase for more to the left- stays the same entire auto

                FrontalLobe.driveController.drive(0, 0, 0);

                while (Utils.timer(1.0, "turret") && opModeIsActive() && !isStopRequested()) {
                    assert Utils.hasTimer("turret");
                    MotorCortex.update();
                    turretSubsystem.update();

                    telemetry.addData("time", (System.nanoTime() / 1E9) - Utils.getTime("turret"));
                    telemetry.update();
                }

                FrontalLobe.useMacro("outtake");
                while (Utils.timer(3.0, "outtake") && opModeIsActive() && !isStopRequested()) {
                    telemetry.addLine("here");
                    telemetry.update();
                    FrontalLobe.update();
                    MotorCortex.update();

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
                pathFollower = TestAutoPathsBlue.getPathFollower(state);
                Utils.clear();
                break;
            case SECOND_PATH:
                state = THIRD_PATH;
                pathFollower = TestAutoPathsBlue.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                break;
            case THIRD_PATH:
                state = FOURTH_PATH;
                pathFollower = TestAutoPathsBlue.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                break;
            case FOURTH_PATH:


                FrontalLobe.driveController.drive(0, 0, 0);

                FrontalLobe.useMacro("outtake");
                while (Utils.timer(3.0, "outtake") && opModeIsActive() && !isStopRequested()) {
                    telemetry.addLine("here");
                    telemetry.update();
                    FrontalLobe.update();
                    MotorCortex.update();

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
                pathFollower = TestAutoPathsBlue.getPathFollower(state);
                Utils.clear();
                break;

            case FIFTH_PATH:
                state = SIXTH_PATH;
                pathFollower = TestAutoPathsBlue.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                break;

            case SIXTH_PATH:
                state = SEVENTH_PATH;
                pathFollower = TestAutoPathsBlue.getPathFollower(state);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                break;
                
            case SEVENTH_PATH:
                state = EIGHTH_PATH;
                pathFollower = TestAutoPathsBlue.getPathFollower(state);

                break;

            case EIGHTH_PATH:
                FrontalLobe.driveController.drive(0, 0, 0);

                FrontalLobe.useMacro("outtake");
                while (Utils.timer(3.0, "outtake") && opModeIsActive() && !isStopRequested()) {
                    telemetry.addLine("here");
                    telemetry.update();
                    FrontalLobe.update();
                    MotorCortex.update();

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

        state = TestAutoPathsBlue.PathState.FIRST_PATH;
        pathFollower = TestAutoPathsBlue.getPathFollower(state);

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

            boolean isStatic = tracker.getRobotVelocity().getMagnitude() < 1.0;
            mecanumController.setIsStatic(isStatic);

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

            if (pathFollower.isFinished(0.2, 0.05) || (currentTime - start) > state.getTimer())
                nextState();

            if (pathFollower != null)
                pathFollower.update();

            telemetry.addData("shooter", outtakeSubsystem.getRPM());
            telemetry.addData("target", outtakeSubsystem.getTargetRPM());
            telemetry.update();
        }
    }
}
