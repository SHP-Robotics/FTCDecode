package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tuning.draw;
import static org.firstinspires.ftc.teamcode.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@Config
@Autonomous(name = "Blue Close Auto - Thanks Angela")
public class BlueCloseAuto extends BaseRobot {
    private PathChain forwards;
    private PathChain forwards_again;
    private PathChain intake_1;
    private PathChain outtake_1;
    private PathChain attack_the_human_move;
    private PathChain attack_the_human_go_time;
    private PathChain return_to_mama;

    enum AutoState {
        MOVING_1 (1.0),
        MOVING_2 (4.0),
        MOVING_3 (4.0),
        MOVING_4 (1.25),
        MOVING_5 (3.0),
        MOVING_6 (3.0),
        MOVING_7 (3.0),
        DONE (Double.POSITIVE_INFINITY);

        AutoState(double timer) {
            this.timer = timer;
        }

        double timer;
    }

    AutoState autoState;
    long state_start;
    long start;

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = false;
        PestoFTCConfig.initializeDrive = false;
        super.initialize();

        FrontalLobe.update();
        MotorCortex.update();

        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.setStartingPose(new Pose(0, 0));

        autoState = AutoState.MOVING_1;

        hoodSubsystem.setState(HoodSubsystem.HoodState.CLOSE);
        hoodSubsystem.update();

        follower.update();
        follower.activateDrive();
        follower.setMaxPower(0.7);

        forwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(0,0), new Pose(4,0)))
                .setLinearHeadingInterpolation(0, -Math.PI/2)
                .build();

        forwards_again = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(4, 0), new Pose(57, 0)))
                .setConstantHeadingInterpolation(-Math.PI/2)
                .build();

        intake_1 = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(57, 0), new Pose(57, -40)))
                .setConstantHeadingInterpolation(-Math.PI/2)
                .build();

        outtake_1 = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(57, -40), new Pose(4, 0)))
                .setLinearHeadingInterpolation(-Math.PI/2, -0.40)
                .build();

        attack_the_human_move = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierCurve(new Pose(4, 0), new Pose(20, 0), new Pose(40, -46.5)))
                .setConstantHeadingInterpolation(-Math.PI)
                .build();

        attack_the_human_go_time = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(40, -46.5), new Pose(8, -46.5)))
                .setConstantHeadingInterpolation(-Math.PI)
                .build();

        return_to_mama = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierCurve(new Pose(8, -46.5), new Pose(32, 0), new Pose(5, 0)))
                .setLinearHeadingInterpolation(-Math.PI, -0.4)
                .build();

        follower.followPath(forwards, true);

        outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_CLOSE);

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("r", follower.getPose().getHeading());
        telemetry.update();

        waitForStart();

        follower.setPose(new Pose(0, 0));

        state_start = System.nanoTime();


        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();

            if (autoState != AutoState.DONE)
                follower.update();

            draw();

            if (((System.nanoTime() - state_start) / 1E9) > autoState.timer) {
                switch (autoState) {
                    case MOVING_1:
                        FrontalLobe.useMacro("auto_outtake");
                        while (opModeIsActive() && !isStopRequested() && FrontalLobe.hasMacro("auto_outtake")) {
                            MotorCortex.update();
                            follower.update();
                            FrontalLobe.update();

                            outtakeSubsystem.update();
                            feederSubsystem.update();
                            intakeSubsystem.update();
                            indexerSubsystem.update();
                        }
                        start = System.nanoTime();
                        while (opModeIsActive() && !isStopRequested() && (System.nanoTime() - start) / 1E9 < 4) {
                            MotorCortex.update();
                            follower.update();
                        }

                        outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                        feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                        indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);

                        outtakeSubsystem.update();
                        feederSubsystem.update();
                        intakeSubsystem.update();
                        indexerSubsystem.update();

                        autoState = AutoState.MOVING_2;
                        state_start = System.nanoTime();
                        follower.followPath(forwards_again, true);
                        break;
                    case MOVING_2:
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                        intakeSubsystem.update();

                        feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                        feederSubsystem.update();

                        autoState = AutoState.MOVING_3;
                        follower.followPath(intake_1, true);
                        follower.setMaxPower(0.5);
                        break;
                    case MOVING_3:
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                        intakeSubsystem.update();

                        feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);
                        feederSubsystem.update();

                        autoState = AutoState.MOVING_4;
                        state_start = System.nanoTime();
                        follower.followPath(outtake_1, true);
                        follower.setMaxPower(1.0);
                        break;
                    case MOVING_4:
                        FrontalLobe.useMacro("auto_outtake");
                        while (opModeIsActive() && !isStopRequested() && FrontalLobe.hasMacro("auto_outtake")) {
                            MotorCortex.update();
                            follower.update();
                            FrontalLobe.update();

                            outtakeSubsystem.update();
                            feederSubsystem.update();
                            intakeSubsystem.update();
                            indexerSubsystem.update();
                        }
                        start = System.nanoTime();
                        while (opModeIsActive() && !isStopRequested() && (System.nanoTime() - start) / 1E9 < 4) {
                            MotorCortex.update();
                            follower.update();
                        }

                        outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                        feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                        indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);

                        outtakeSubsystem.update();
                        feederSubsystem.update();
                        intakeSubsystem.update();
                        indexerSubsystem.update();

                        state_start = System.nanoTime();
                        follower.followPath(attack_the_human_move, true);
                        autoState = AutoState.MOVING_5;
                        break;
                    case MOVING_5:
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);
                        intakeSubsystem.update();

                        feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                        feederSubsystem.update();

                        follower.followPath(attack_the_human_go_time, true);
                        follower.setMaxPower(0.5);

                        state_start = System.nanoTime();
                        autoState = AutoState.MOVING_6;
                        break;
                    case MOVING_6:
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                        intakeSubsystem.update();

                        feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);
                        feederSubsystem.update();

                        follower.followPath(return_to_mama, true);
                        follower.setMaxPower(1.0);

                        state_start = System.nanoTime();
                        autoState = AutoState.MOVING_7;
                        break;
                    case MOVING_7:
                        FrontalLobe.useMacro("auto_outtake");
                        while (opModeIsActive() && !isStopRequested() && FrontalLobe.hasMacro("auto_outtake")) {
                            MotorCortex.update();
                            follower.update();
                            FrontalLobe.update();

                            outtakeSubsystem.update();
                            feederSubsystem.update();
                            intakeSubsystem.update();
                            indexerSubsystem.update();
                        }
                        start = System.nanoTime();
                        while (opModeIsActive() && !isStopRequested() && (System.nanoTime() - start) / 1E9 < 4) {
                            MotorCortex.update();
                            follower.update();
                        }

                        outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.NEUTRAL);
                        feederSubsystem.setState(FeederSubsystem.FeederState.STOPPED);
                        intakeSubsystem.setState(IntakeSubsystem.IntakeState.NEUTRAL);
                        indexerSubsystem.setState(IndexerSubsystem.IndexerState.NEUTRAL);

                        outtakeSubsystem.update();
                        feederSubsystem.update();
                        intakeSubsystem.update();
                        indexerSubsystem.update();

                        state_start = System.nanoTime();
                        autoState = AutoState.DONE;
                        break;
                }
            }

            telemetry.addData("time elapsed", ((System.nanoTime() - state_start) / 1E9));
            telemetry.addData("timer", autoState.timer);
            telemetry.addData("state", autoState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("r", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}