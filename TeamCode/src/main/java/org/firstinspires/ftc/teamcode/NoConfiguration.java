package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp(name = "No Configuration")
public class NoConfiguration extends LinearOpMode {
    public DcMotor getMotor(int port) {
        // From HardwareFactory.java
        LynxDcMotorController controller;
        try {
            controller = new LynxDcMotorController(hardwareMap.appContext, hardwareMap.getAll(LynxModule.class).get(0));
        }  catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }

        // USB Scan Manager.java uses null manager?
        DeviceManager deviceMgr = new HardwareDeviceManager(hardwareMap.appContext, null);

        // Using GoBILDA 5203 Motor Configuration
        MotorConfigurationType motorConfigurationType = new MotorConfigurationType();
        motorConfigurationType.setTicksPerRev(505.3169);
        motorConfigurationType.setGearing(99.5);
        motorConfigurationType.setMaxRPM(60);
        motorConfigurationType.setOrientation(Rotation.CCW);

        DcMotor m = deviceMgr.createDcMotorEx(controller, port, motorConfigurationType, motorConfigurationType.getName());

        // Since it is not automatically enabled, we manually enable it
        MotorCortex.MotorCommands.enableMotor(m);

        return m;
    }

    @Override
    public void runOpMode() {
        DcMotor port0 = getMotor(0);
        DcMotor port1 = getMotor(1);
        DcMotor port2 = getMotor(2);
        DcMotor port3 = getMotor(3);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("port - 0", port0.getCurrentPosition());
            telemetry.addData("port - 1", port1.getCurrentPosition());
            telemetry.addData("port - 2", port2.getCurrentPosition());
            telemetry.addData("port - 3", port3.getCurrentPosition());
            telemetry.update();
        }
    }
}
