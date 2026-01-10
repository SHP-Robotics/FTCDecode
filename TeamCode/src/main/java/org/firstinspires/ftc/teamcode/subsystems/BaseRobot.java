package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public BlockerSubsystem blockerSubsystem;
    public HoodSubsystem hoodSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public OuttakeSubsystem outtakeSubsystem;
    public IndexerSubsystem indexerSubsystem;
    public TurretSubsystem turretSubsystem;

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
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (PestoFTCConfig.initializePinpoint) {
            tracker = FrontalLobe.tracker;
            tracker.reset();

            teleOpController = FrontalLobe.teleOpController;
        }

        blockerSubsystem = new BlockerSubsystem();
        hoodSubsystem = new HoodSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        outtakeSubsystem = new OuttakeSubsystem();
        indexerSubsystem = new IndexerSubsystem();
        turretSubsystem = new TurretSubsystem();

        gamepadInterface1 = new GamepadInterface(gamepad1);

        // State initialization
        state = RobotState.NEUTRAL;
        brake = false;

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
                if (v < 1.5)
                    return false;

                blockerSubsystem.setState(BlockerSubsystem.BlockerState.OUTTAKE);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTTAKE);

                return true;
            }
        });
    }

    @Override
    public void runOpMode() {

    }
}
