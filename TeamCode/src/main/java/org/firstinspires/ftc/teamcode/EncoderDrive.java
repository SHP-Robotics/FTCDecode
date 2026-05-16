package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MecanumEncoderTeleOp")
public class EncoderDrive extends LinearOpMode {
    DcMotor frontLeft, backLeft, frontRight, backRight;
    // goBILDA 435 RPM + 96mm Wheel constant
    final double TICKS_PER_INCH = 32.4;

    @Override
    public void runOpMode() {
        // ... (Your hardware mapping from original code) ...

        waitForStart();

        while (opModeIsActive()) {
            // Standard Mecanum TeleOp Control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Trigger "Auto-Drive 10 inches" when 'A' is pressed
            if (gamepad1.a) {
                driveInches(10, 0.4);
            }

            // Normal driving logic (Mecanum Math)
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft.setPower((y + x + rx) / denominator);
            backLeft.setPower((y - x + rx) / denominator);
            frontRight.setPower((y - x - rx) / denominator);
            backRight.setPower((y + x - rx) / denominator);
        }
    }

    // Helper method to move specific distance
    public void driveInches(double inches, double power) {
        int target = (int)(inches * TICKS_PER_INCH);

        // Reset and set targets
        DcMotor[] motors = {frontLeft, backLeft, frontRight, backRight};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(target);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        // Wait until reached
        while (opModeIsActive() && frontLeft.isBusy()) {
            telemetry.addData("Moving to", target);
            telemetry.addData("Current", frontLeft.getCurrentPosition());
            telemetry.update();
        }

        // Set back to regular driving mode
        for (DcMotor m : motors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}