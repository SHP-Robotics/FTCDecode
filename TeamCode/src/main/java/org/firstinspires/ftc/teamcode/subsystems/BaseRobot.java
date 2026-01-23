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

        mecanumController = (MecanumController) FrontalLobe.driveController;
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        teleOpController = FrontalLobe.teleOpController;

        if (PestoFTCConfig.initializePinpoint) {
            tracker = FrontalLobe.tracker;
            tracker.reset();
        }

        gamepadInterface1 = new GamepadInterface(gamepad1);

        // State initialization
        state = RobotState.NEUTRAL;
    }

    @Override
    public void runOpMode() {

    }
}