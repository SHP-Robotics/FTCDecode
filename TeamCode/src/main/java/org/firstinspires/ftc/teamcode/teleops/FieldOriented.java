package org.firstinspires.ftc.teamcode.teleops;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

@TeleOp(name = "Field Oriented")
public class FieldOriented extends LinearOpMode {
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotorEx leftIntake;
    DcMotorEx rightIntake;

    DcMotorEx transfer;
    DcMotorEx outtake;

    private static final String WEBCAM_NAME = "Webcam 1";
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private String cameraError;
    private final AprilTagAlignmentController alignment = new AprilTagAlignmentController();

    private double heading;
    private double zero;

    public void driveFieldOriented(double x, double y, double turn, double heading) {
        double cosHeading = Math.cos(heading);
        double sinHeading = Math.sin(heading);

        double strafe = x * cosHeading + y * sinHeading;
        double forward = -x * sinHeading + y * cosHeading;

        // The same robot-relative mecanum mixing as BasicMecanumTeleOp.
        double leftFrontPower = forward + strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double leftBackPower = forward - strafe + turn;
        double rightBackPower = forward + strafe - turn;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftIntake = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        rightIntake = hardwareMap.get(DcMotorEx.class, "intakeRight");

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

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

        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        heading = 0;
        zero = 0;

        telemetry.addLine("Initializing...");
        telemetry.update();

        initAprilTag();

        telemetry.addLine("Camera Initialized...");
        telemetry.update();

        while (opModeInInit()) {
            sleep(10);
            // can add logging code here, before the opmode has started
        }

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.y)
                zero = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            List<AprilTagDetection> detections = getDetections();
            Double autoTurn = alignment.update(detections);
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - zero;
            driveFieldOriented(gamepad1.left_stick_x, -gamepad1.left_stick_y, (gamepad1.b && autoTurn != null) ? autoTurn : gamepad1.right_stick_x, heading);

            double intakePower = gamepad1.left_trigger * (gamepad1.left_bumper ? -1.0 : 1.0);
            leftIntake.setPower(intakePower);
            rightIntake.setPower(intakePower);

            telemetry.addData("Speed", gamepad1.b ? "Auto (12-25% turn)" : gamepad1.left_bumper ? "Slow" : "Full");
            telemetry.addData("Heading (not deg)", "%.1f", heading);
            telemetry.addLine();
            addVisionTelemetry(detections);
            telemetry.update();

            transfer.setPower(gamepad1.right_trigger);
            outtake.setPower(gamepad1.right_trigger * .50);
        }
    }

    private void initAprilTag() {
        try {
            AprilTagLibrary.Builder tagLibrary = new AprilTagLibrary.Builder()
                    .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
            // Measured black-square width, excluding the surrounding white margin.
            for (int id = 30; id <= 45; id++) {
                tagLibrary.addTag(id, "Tag " + id, 3.25, DistanceUnit.INCH);
            }
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagLibrary(tagLibrary.build())
                    .setDrawTagOutline(true)
                    .setDrawTagID(true)
                    .setDrawAxes(true)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();
            // Uses built-in webcam calibration, when available.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(true)
                    .setAutoStopLiveView(false)
                    .addProcessor(aprilTag)
                    .build();
        } catch (RuntimeException exception) {
            cameraError = "Check " + WEBCAM_NAME + ": " + exception.getMessage();
            telemetry.log().add("Vision unavailable: " + cameraError);
        }
    }

    private boolean isCameraStreaming() {
        return visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    private List<AprilTagDetection> getDetections() {
        return isCameraStreaming() ? aprilTag.getDetections() : Collections.emptyList();
    }

    private void addVisionTelemetry(List<AprilTagDetection> detections) {
        telemetry.addData("Camera", cameraError != null ? cameraError
                : visionPortal == null ? "Unavailable" : visionPortal.getCameraState());
        telemetry.addData("AprilTag alignment", alignment.getStatus());
        telemetry.addData("Tags in latest frame", detections.size());
        AprilTagDetection target = AprilTagAlignmentController.selectTarget(detections, alignment.getTargetId());
        if (target != null) {
            telemetry.addData("Tag yaw / pitch / roll", "%.1f / %.1f / %.1f deg", target.ftcPose.yaw, target.ftcPose.pitch, target.ftcPose.roll);
        } else {
            telemetry.addData("Target", "No fresh, known tag visible");
        }
//        for (AprilTagDetection detection : detections) {
//            if (detection.metadata == null) {
//                telemetry.addData("Tag " + detection.id, "Unknown tag - add ID and size to tag library");
//            } else if (detection.ftcPose == null) {
//                telemetry.addData("Tag " + detection.id, "Known size, but no pose available - cannot align");
//            }
//        }
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
