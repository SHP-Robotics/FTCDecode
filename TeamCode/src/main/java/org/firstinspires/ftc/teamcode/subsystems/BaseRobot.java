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

    public IntakeSubsystem intakeSubsystem;
    public FeederSubsystem feederSubsystem;
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
//            tracker = FrontalLobe.tracker;
//            tracker.reset();

            teleOpController = FrontalLobe.teleOpController;
        }

        intakeSubsystem = new IntakeSubsystem();
        feederSubsystem = new FeederSubsystem();
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
                if (v < 0.75)
                    return false;

                feederSubsystem.setState(FeederSubsystem.FeederState.INTAKE);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.INTAKE);

                return true;
            }
        });

        FrontalLobe.addMacro("giggity", new FrontalLobe.Macro() {
            @Override
            public void start() {
                FrontalLobe.removeOtherMacros(this);
                teleOpController.driveRobotCentric(1.0, 0, 0);
            }

            @Override
            public boolean loop(double v) {
                // how long (seconds) before starting to move other components
                if (v < 0.2)
                    return false;

                teleOpController.driveRobotCentric(-0.3, 0, 0);

                if (v < 0.3)
                    return false;

                return true;
            }
        });
    }
}
