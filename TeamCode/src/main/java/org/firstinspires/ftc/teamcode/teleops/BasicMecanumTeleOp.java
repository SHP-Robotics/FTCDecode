package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic Mecanum TeleOp", group = "Drive")
public class BasicMecanumTeleOp extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");

        // Reverse the left intake to account for its mounting.
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            runDrive();
        } finally {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }

    private void runDrive() {
        telemetry.addLine("Ready");
        telemetry.addData("Motors", "leftFront, rightFront, leftBack, rightBack");
        telemetry.addLine("Left trigger: both intakes | Hold left bumper + trigger: outtake");
        telemetry.addLine("Left bumper also sets drive speed to 35%");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double leftFrontPower = forward + strafe + turn;
            double rightFrontPower = forward - strafe - turn;
            double leftBackPower = forward - strafe + turn;
            double rightBackPower = forward + strafe - turn;

            double max = Math.max(1.0, Math.abs(leftFrontPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            double speedMultiplier = gamepad1.left_bumper ? 0.35 : 1.0;

            leftFront.setPower((leftFrontPower / max) * speedMultiplier);
            rightFront.setPower((rightFrontPower / max) * speedMultiplier);
            leftBack.setPower((leftBackPower / max) * speedMultiplier);
            rightBack.setPower((rightBackPower / max) * speedMultiplier);

            double intakePower = gamepad1.left_trigger * (gamepad1.left_bumper ? -1.0 : 1.0);
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);

            telemetry.addData("Speed", gamepad1.left_bumper ? "Slow" : "Full");
            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Intake power", "%.2f", intakePower);
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
