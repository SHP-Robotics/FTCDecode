package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;

public class BaseRobot extends LinearOpMode {
    public MecanumController mecanumController;
    public TeleOpController teleOpController;
    public DeterministicTracker tracker;

    public BlockerSubsystem blockerSubsystem;
    public HoodSubsystem hoodSubsystem;
    public IntakeOuttakeSubsystem intakeOuttakeSubsystem;

    public GamepadInterface gamepadInterface1;

    public RobotState state;
    public boolean brake;

    public enum RobotState {
        INTAKE,
        OUTTAKE,
        REJECT,
        NEUTRAL
    }

    public void initialize() {
        FrontalLobe.initialize(hardwareMap);

        mecanumController = (MecanumController) FrontalLobe.driveController;
//        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        teleOpController = FrontalLobe.teleOpController;

        tracker = FrontalLobe.tracker;
        tracker.reset();

        blockerSubsystem = new BlockerSubsystem();
        hoodSubsystem = new HoodSubsystem();
        intakeOuttakeSubsystem = new IntakeOuttakeSubsystem();

        gamepadInterface1 = new GamepadInterface(gamepad1);

        // State initialization
        state = RobotState.NEUTRAL;
        brake = false;

        // MACRO initialization

        FrontalLobe.addMacro("outtake", new FrontalLobe.Macro() {
            @Override
            public void start() {
                FrontalLobe.removeOtherMacros(this);
                intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.PREREV);
            }

            @Override
            public boolean loop(double v) {
                // how long (seconds) before starting to move other components
                if (v < 0.3)
                    return false;
                intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.REV);
                if (v < 2)
                    return false;

                blockerSubsystem.setState(BlockerSubsystem.BlockerState.NEUTRAL);
                intakeOuttakeSubsystem.setState(IntakeOuttakeSubsystem.IntakeOuttakeState.OUTTAKE);


                return true;
            }
        });
    }

    @Override
    public void runOpMode() {

    }
}
