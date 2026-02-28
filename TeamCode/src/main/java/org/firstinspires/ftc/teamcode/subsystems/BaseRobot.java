package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

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
    public CortexLinkedServo dogGear;
    public CortexLinkedServo LED;

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
        dogGear = MotorCortex.getServo(0, 173); //MotorCortex.getServo("dog");
        LED = MotorCortex.getServo(0, 2);

        gamepadInterface1 = new GamepadInterface(gamepad1);

        // State initialization
        state = RobotState.NEUTRAL;
        brake = false;

        // MACRO initialization
    }

    @Override
    public void runOpMode() {

    }
}
