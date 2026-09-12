package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.List;

@TeleOp(name = "Motor Port Finder", group = "Utility")
public class MotorPortFinderTeleOp extends LinearOpMode {
    private static final int FIRST_MOTOR_PORT = 0;
    private static final int LAST_MOTOR_PORT = 3;
    private static final double TEST_POWER = 0.25;
    private static final String UNKNOWN_WHEEL = "not found yet";

    @Override
    public void runOpMode() {
        List<DcMotorControllerEx> controllers = hardwareMap.getAll(DcMotorControllerEx.class);

        if (controllers.isEmpty()) {
            telemetry.addLine("No motor controller found.");
            telemetry.addLine("Make sure the Control Hub is in the active robot config.");
            telemetry.update();
            return;
        }

        int selectedController = 0;
        int selectedPort = FIRST_MOTOR_PORT;
        String[][] wheelByControllerAndPort = new String[controllers.size()][LAST_MOTOR_PORT + 1];

        boolean previousA = false;
        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;
        boolean previousDpadRight = false;
        boolean previousDpadLeft = false;

        for (int controllerIndex = 0; controllerIndex < wheelByControllerAndPort.length; controllerIndex++) {
            for (int port = FIRST_MOTOR_PORT; port <= LAST_MOTOR_PORT; port++) {
                wheelByControllerAndPort[controllerIndex][port] = UNKNOWN_WHEEL;
            }
        }

        for (DcMotorControllerEx controller : controllers) {
            setupController(controller);
        }

        telemetry.addLine("Ready");
        telemetry.addLine("This tests Control Hub motor ports directly.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            DcMotorControllerEx controller = controllers.get(selectedController);

            if (gamepad1.dpad_right && !previousDpadRight) {
                selectedPort++;
                if (selectedPort > LAST_MOTOR_PORT) {
                    selectedPort = FIRST_MOTOR_PORT;
                }
            }

            if (gamepad1.dpad_left && !previousDpadLeft) {
                selectedPort--;
                if (selectedPort < FIRST_MOTOR_PORT) {
                    selectedPort = LAST_MOTOR_PORT;
                }
            }

            if (gamepad1.dpad_up && !previousDpadUp) {
                selectedController++;
                if (selectedController >= controllers.size()) {
                    selectedController = 0;
                }
            }

            if (gamepad1.dpad_down && !previousDpadDown) {
                selectedController--;
                if (selectedController < 0) {
                    selectedController = controllers.size() - 1;
                }
            }

            if (gamepad1.a && !previousA) {
                wheelByControllerAndPort[selectedController][selectedPort] = "front left";
            }

            if (gamepad1.b && !previousB) {
                wheelByControllerAndPort[selectedController][selectedPort] = "front right";
            }

            if (gamepad1.x && !previousX) {
                wheelByControllerAndPort[selectedController][selectedPort] = "back left";
            }

            if (gamepad1.y && !previousY) {
                wheelByControllerAndPort[selectedController][selectedPort] = "back right";
            }

            previousA = gamepad1.a;
            previousB = gamepad1.b;
            previousX = gamepad1.x;
            previousY = gamepad1.y;
            previousDpadRight = gamepad1.dpad_right;
            previousDpadLeft = gamepad1.dpad_left;
            previousDpadUp = gamepad1.dpad_up;
            previousDpadDown = gamepad1.dpad_down;

            stopAllControllers(controllers);

            double power = 0.0;
            if (gamepad1.right_trigger > 0.2) {
                power = TEST_POWER;
            } else if (gamepad1.left_trigger > 0.2) {
                power = -TEST_POWER;
            }

            controller.setMotorPower(selectedPort, power);

            telemetry.addLine("MOTOR PORT FINDER");
            telemetry.addData("Controller", getDeviceName(controller));
            telemetry.addData("Controller index", "%d of %d", selectedController + 1, controllers.size());
            telemetry.addData("Selected motor port", selectedPort);
            telemetry.addLine("");
            telemetry.addData("D-pad left/right", "change motor port");
            telemetry.addData("D-pad up/down", "change hub/controller");
            telemetry.addData("Right trigger", "spin forward");
            telemetry.addData("Left trigger", "spin backward");
            telemetry.addData("A", "mark as front left");
            telemetry.addData("B", "mark as front right");
            telemetry.addData("X", "mark as back left");
            telemetry.addData("Y", "mark as back right");
            telemetry.addLine("");
            telemetry.addLine("CURRENT MAP");
            telemetry.addData("Port 0", wheelByControllerAndPort[selectedController][0]);
            telemetry.addData("Port 1", wheelByControllerAndPort[selectedController][1]);
            telemetry.addData("Port 2", wheelByControllerAndPort[selectedController][2]);
            telemetry.addData("Port 3", wheelByControllerAndPort[selectedController][3]);
            telemetry.update();
        }

        stopAllControllers(controllers);
    }

    private void setupController(DcMotorControllerEx controller) {
        for (int port = FIRST_MOTOR_PORT; port <= LAST_MOTOR_PORT; port++) {
            controller.setMotorMode(port, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            controller.setMotorZeroPowerBehavior(port, DcMotor.ZeroPowerBehavior.BRAKE);
            controller.setMotorPower(port, 0.0);
        }
    }

    private void stopAllControllers(List<DcMotorControllerEx> controllers) {
        for (DcMotorControllerEx controller : controllers) {
            for (int port = FIRST_MOTOR_PORT; port <= LAST_MOTOR_PORT; port++) {
                controller.setMotorPower(port, 0.0);
            }
        }
    }

    private String getDeviceName(HardwareDevice device) {
        StringBuilder name = new StringBuilder();

        for (String deviceName : hardwareMap.getNamesOf(device)) {
            if (name.length() > 0) {
                name.append(", ");
            }
            name.append(deviceName);
        }

        if (name.length() == 0) {
            return device.getDeviceName();
        }

        return name.toString();
    }
}
