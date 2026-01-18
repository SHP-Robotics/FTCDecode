package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.TestAutoPathsBlue.PathState.DONE;

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

@Autonomous(name = "New auto")
public class NewAuto extends BaseRobot {
    TestAutoPathsBlue.PathState state;
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
                    blockerSubsystem.setState(org.firstinspires.ftc.teamcode.subsystems.BlockerSubsystem.BlockerState.BLOCK);

                    state = TestAutoPathsBlue.PathState.SECOND_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
                    Utils.clear();
                    break;
                case SECOND_PATH:
                    state = TestAutoPathsBlue.PathState.THIRD_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                    break;
                case THIRD_PATH:
                    state = TestAutoPathsBlue.PathState.FOURTH_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
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

                    state = TestAutoPathsBlue.PathState.FIFTH_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
                    Utils.clear();
                    break;

                case FIFTH_PATH:
                    state = TestAutoPathsBlue.PathState.SIXTH_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                    break;

                case SIXTH_PATH:
                    state = TestAutoPathsBlue.PathState.SEVENTH_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
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

                    state = TestAutoPathsBlue.PathState.EIGHTH_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);

                    break;

                case EIGHTH_PATH:
                    state = TestAutoPathsBlue.PathState.NINETH_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                    break;

                case NINETH_PATH:
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
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

                    state = TestAutoPathsBlue.PathState.TENTH_PATH;
                    pathFollower = TestAutoPathsBlue.getPathFollower(state);
                    break;

                case TENTH_PATH:
                    intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                    state = TestAutoPathsBlue.PathState.DONE;
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

            if (pathFollower.isFinished() || (currentTime - start) > state.getTimer())
                nextState();

            if (pathFollower != null)
                pathFollower.update();

            telemetry.addData("shooter", outtakeSubsystem.getRPM());
            telemetry.addData("target", outtakeSubsystem.getTargetRPM());
            telemetry.update();
        }
    }
}
