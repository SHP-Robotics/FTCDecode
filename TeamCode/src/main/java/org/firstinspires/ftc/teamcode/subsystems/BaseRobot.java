package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    public IndexerSubsystem indexerSubsystem;

    public Limelight3A limelight;

    public GamepadInterface gamepadInterface1;

    public RobotState state;

    public enum RobotState {
        INTAKE,
        OUTTAKE,
        REJECT,
        NEUTRAL
    }

    public void initialize() {
        FrontalLobe.initialize(hardwareMap);

        if (PestoFTCConfig.initializeDrive)
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
//        indexerSubsystem = new IndexerSubsystem();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

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
                if (v < 1.0)
                    return false;

                feederSubsystem.setState(FeederSubsystem.FeederState.FORWARD);
                intakeSubsystem.setState(IntakeSubsystem.IntakeState.OUTTAKE);
//                indexerSubsystem.setState(IndexerSubsystem.IndexerState.OUTTAKE);

                return true;
            }
        });

//        FrontalLobe.addMacro("limelight - align", new FrontalLobe.Macro() {
//            @Override
//            public void start() {
//
//            }
//
//            @Override
//            public boolean loop(double v) {
//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.update();
//
//                    double rotate = -result.getTx() * PestoFTCConfig.KP;
//                    rotate = Math.min(1, Math.max(-1, rotate));
//
//                    if (rotate < 0)
//                        rotate -= PestoFTCConfig.STATIC_DRIVE;
//                    else
//                        rotate += PestoFTCConfig.STATIC_DRIVE;
//
//                    teleOpController.driveRobotCentric(0, 0, rotate);
//                }
//
//                return v > 2.0;
//            }
//        });
    }

    @Override
    public void runOpMode() {

    }
}