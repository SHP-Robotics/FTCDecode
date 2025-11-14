package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class BaseRobot extends LinearOpMode {
    public MecanumController mecanumController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;

    public FeederSubsystem feederSubsystem;
    public HoodSubsystem hoodSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;

    public GamepadInterface gamepadInterface1;

    public RobotState state;

    public enum RobotState {
        INTAKE,
        OUTTAKE,
        REJECT,
        NEUTRAL
    }

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        mecanumController = (MecanumController) FrontalLobe.driveController;
        if (PestoFTCConfig.initializePinpoint) {
            tracker = FrontalLobe.tracker;
            tracker.reset();

            teleOpController = FrontalLobe.teleOpController;
        }

        feederSubsystem = new FeederSubsystem();
        hoodSubsystem = new HoodSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        outtakeSubsystem = new OuttakeSubsystem();

        gamepadInterface1 = new GamepadInterface(gamepad1);

        // State initialization
        state = RobotState.NEUTRAL;

        // MACRO initialization

        FrontalLobe.addMacro("outtake", new FrontalLobe.Macro() {
            @Override
            public void start() {
                FrontalLobe.removeOtherMacros(this);
                outtakeSubsystem.setState(OuttakeSubsystem.OuttakeState.OUTTAKE);
            }

            @Override
            public boolean loop(double v) {
                // how long (seconds) before starting to move other components
                if (v < 0.5)
                    return false;

                feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTTAKE);

                return true;
            }
        });
    }
}
