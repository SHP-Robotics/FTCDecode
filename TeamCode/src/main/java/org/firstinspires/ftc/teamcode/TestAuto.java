package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tuning.draw;
import static org.firstinspires.ftc.teamcode.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.PanelsConfigurables;
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
@Autonomous(name = "Hell no")
public class TestAuto extends BaseRobot {
    private PathChain forwards;
    private PathChain forwards_again;

    public static double forward_dist = 27;

    enum AutoState {
        MOVING_1 (Double.POSITIVE_INFINITY),
        MOVING_2 (10.0),
        DONE (Double.POSITIVE_INFINITY);

        AutoState(double timer) {
            this.timer = timer;
        }

        double timer;
    }

    AutoState autoState;
    long state_start;

    @Override
    public void init() {
        PestoFTCConfig.initializePinpoint = false;
        PestoFTCConfig.initializeDrive = false;
        super.init();

        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);

        follower.setStartingPose(new Pose(0, 0));

        autoState = AutoState.MOVING_1;

        hoodSubsystem.setState(HoodSubsystem.HoodState.AUTO_FAR);
        hoodSubsystem.update();
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        super.init();

        follower.update();
    }

    @Override
    public void start() {
        follower.activateDrive();

        follower.setMaxPower(0.7);

        forwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(0,0), new Pose(4,0)))
                .setLinearHeadingInterpolation(0, -0.22)
                .build();

        forwards_again = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(4, 0), new Pose(forward_dist, 0)))
                .setLinearHeadingInterpolation(-0.22, -Math.PI/2)
                .build();

        follower.followPath(forwards);

        outtakeSubsystem.setPower(PestoFTCConfig.SHOOTER_FAR);

        state_start = System.nanoTime();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        FrontalLobe.update();
        MotorCortex.update();

        if (autoState != AutoState.DONE)
            follower.update();

        draw();

        if (!follower.isBusy() || ((System.nanoTime() - state_start) / 1E9) > autoState.timer) {
            switch (autoState) {
                case MOVING_1:
                    FrontalLobe.useMacro("outtake");
                    while (FrontalLobe.hasMacro("outtake")) {
                        FrontalLobe.update();

                        outtakeSubsystem.update();
                        feederSubsystem.update();
                        intakeSubsystem.update();
                        indexerSubsystem.update();
                    }
                    long start = System.nanoTime();
                    while ((System.nanoTime() - start) / 1E9 < 5) {}

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
                    follower.followPath(forwards_again);
                    break;
                case MOVING_2:
                    autoState = AutoState.DONE;
            }
        }

        telemetry.addData("time elapsed", ((System.nanoTime() - state_start) / 1E9));
        telemetry.addData("timer", autoState.timer);
        telemetry.addData("state", autoState);
        telemetry.update();
    }
}