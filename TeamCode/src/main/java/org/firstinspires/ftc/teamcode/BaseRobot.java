package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.processing.FrontalLobe;

public class BaseRobot extends LinearOpMode {
    public MecanumController mecanumController;
    public TeleOpController teleOpController;

    public GamepadInterface gamepadInterface1;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        mecanumController = (MecanumController) FrontalLobe.driveController;
        teleOpController = FrontalLobe.teleOpController;

        gamepadInterface1 = new GamepadInterface(gamepad1);

    }
}