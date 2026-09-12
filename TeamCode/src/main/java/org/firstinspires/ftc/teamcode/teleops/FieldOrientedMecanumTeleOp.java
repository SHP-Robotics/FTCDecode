package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Left stick moves relative to the field; right stick turns the robot.
 * Face the desired field-forward direction before pressing START.
 * Press Y to make the robot's current facing direction the new field-forward direction.
 * Hold the left bumper for 35% speed, just like BasicMecanumTeleOp.
 * Requires an IMU named "imu" and hub orientation settings that match its physical mounting.
 */
@TeleOp(name = "Field Oriented Mecanum TeleOp", group = "Drive")
public class FieldOrientedMecanumTeleOp extends LinearOpMode {
    // Hub mounting relative to the robot: REV logo faces left, USB ports face forward.
    // Update these if the hub containing the IMU is remounted.
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);
        if (!imu.initialize(new IMU.Parameters(orientationOnRobot))) {
            telemetry.addLine("IMU initialization failed. Check the IMU and robot configuration.");
            telemetry.update();
            return;
        }

        telemetry.addLine("Ready - face field-forward before pressing START");
        telemetry.addData("Motors", "leftFront, rightFront, leftBack, rightBack");
        telemetry.addData("Hub orientation", "Logo=%s, USB=%s",
                LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);
        telemetry.addLine("Left stick: field movement | Right stick: turn");
        telemetry.addLine("Left bumper: slow | Y: reset field-forward to current heading");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // The IMU does not know field-forward automatically; establish it at START.
        imu.resetYaw();
        boolean previousY = gamepad1.y;

        while (opModeIsActive()) {
            // Reset once per press so holding Y does not continually erase heading changes.
            boolean resetHeading = gamepad1.y;
            if (resetHeading && !previousY) {
                imu.resetYaw();
            }
            previousY = resetHeading;

            double fieldForward = -gamepad1.left_stick_y;
            double fieldStrafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // FTC yaw is positive counterclockwise. Math.sin/cos expect radians.
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double cosHeading = Math.cos(heading);
            double sinHeading = Math.sin(heading);

            // Rotate the field vector by -heading to express it in the robot's frame.
            // x = strafe right, y = forward. At +90 degrees, field-forward is robot-right.
            double strafe = fieldStrafe * cosHeading + fieldForward * sinHeading;
            double forward = -fieldStrafe * sinHeading + fieldForward * cosHeading;

            // The same robot-relative mecanum mixing as BasicMecanumTeleOp.
            double leftFrontPower = forward + strafe + turn;
            double rightFrontPower = forward - strafe - turn;
            double leftBackPower = forward - strafe + turn;
            double rightBackPower = forward + strafe - turn;

            // Scale all wheels together to preserve direction when any power exceeds 1.
            double max = Math.max(1.0, Math.abs(leftFrontPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            double speedMultiplier = gamepad1.left_bumper ? 0.35 : 1.0;

            leftFront.setPower((leftFrontPower / max) * speedMultiplier);
            rightFront.setPower((rightFrontPower / max) * speedMultiplier);
            leftBack.setPower((leftBackPower / max) * speedMultiplier);
            rightBack.setPower((rightBackPower / max) * speedMultiplier);

            telemetry.addData("Speed", gamepad1.left_bumper ? "Slow" : "Full");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("Field forward / strafe", "%.2f / %.2f", fieldForward, fieldStrafe);
            telemetry.addData("Robot forward / strafe", "%.2f / %.2f", forward, strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addLine("Y: reset field-forward to current heading");
            telemetry.update();
        }
    }

    private void setDriveMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
