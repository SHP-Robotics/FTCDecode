package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.POSITION;
import static java.util.Arrays.asList;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class TurretSubsystem {
    public CortexLinkedMotor turret;

    private final PID positionPIDControllerPrimary;
    private final PID positionPIDControllerSecondary;
    public final PID cameraPIDController;

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    private List<Integer> acceptedTags;
    private boolean detectAprilTag;
    private boolean recenterAfter;

    public AprilTagPoseFtc lastAprilTag;
    private long lastAprilTagTimer = 0;
    public boolean detected;

    private double visionOffset = 0.0;

    private TurretState state;

    private double targetPosition;

    public enum TurretState {
        POSITION,
        MANUAL
    }

    public TurretSubsystem() {
        turret = MotorCortex.getMotor("turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(RUN_USING_ENCODER);

        positionPIDControllerPrimary = new PID(PestoFTCConfig.TURRET_KP_PRIMARY, 0, 0);
        positionPIDControllerSecondary = new PID(PestoFTCConfig.TURRET_KP_SECONDARY, 0, 0);
        cameraPIDController = new PID(0.018, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(MotorCortex.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // BLUE GOAL, RED GOAL
        acceptedTags = asList(20, 24);
        detectAprilTag = false;

        lastAprilTag = null;
        detected = false;
        recenterAfter = true;

        state = MANUAL;
    }

    public void reinitialize() {
        turret = MotorCortex.getMotor(3, 2);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(RUN_USING_ENCODER);
    }

    public void setVisionOffset(double visionOffset) {
        this.visionOffset = visionOffset;
    }

    public void setAcceptedTags(List<Integer> tags) {
        this.acceptedTags = tags;
    }

    public void setDetectAprilTag(boolean detectAprilTag) {
        this.detectAprilTag = detectAprilTag;
    }

    public void setRecenterAfter(boolean recenterAfter) {
        this.recenterAfter = recenterAfter;
    }

    public void setPower(double power) {
        this.state = MANUAL;
        turret.setPowerResult(power);
    }

    public void setTargetPosition(double position) {
        this.state = POSITION;
        this.targetPosition = position;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getPosition() {
        return turret.getCurrentPosition();
    }

    public TurretState getState() {
        return state;
    }

    public void rezero() {
        turret.setMode(STOP_AND_RESET_ENCODER);
        turret.setMode(RUN_USING_ENCODER);
    }

    private void updateTags() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null)
                continue;

            if (!acceptedTags.contains(detection.id))
                continue;

            lastAprilTag = detection.ftcPose;
            lastAprilTagTimer = System.nanoTime();
            detected = true;
            return;
        }

        detected = false;
    }

    public double getDegrees() {
        return this.getPosition() * 45 / 310;
    }

    public void setBearing(double bearing) {
        double turretDegrees = this.getDegrees();
        double power = cameraPIDController.getOutput(turretDegrees, turretDegrees + bearing);

        // Left end stop
        if (this.getPosition() > targetPosition + 1860)
            power = Math.min(power, 0);

        // Right end stop
        if (this.getPosition() < targetPosition - 620)
            power = Math.max(power, 0);

        turret.setPowerResult(power);
    }

    public boolean useSecondary() {
        return Math.abs(turret.getCurrentPosition() - targetPosition) < PestoFTCConfig.TURRET_KP_SWITCH;
    }

    public void update() {
        // Camera Processing Code
        updateTags();

        if (detectAprilTag && lastAprilTag != null) {
            if (detected) {
                this.setBearing(lastAprilTag.bearing - visionOffset);
                return;
            }

            // So far zone can recognize
            if (!recenterAfter || (System.nanoTime() - lastAprilTagTimer) / 1E9 < 0.5) {
                turret.setPowerResult(0.0);
                return;
            }
        }

        if (this.state == MANUAL)
            return;

        if (!recenterAfter)
            return;

        // State == POSITION
        if (useSecondary()) {
            double power = positionPIDControllerSecondary.getOutput(turret.getCurrentPosition(), targetPosition);
            turret.setPowerResult(power);
        } else {
            double power = positionPIDControllerPrimary.getOutput(turret.getCurrentPosition(), targetPosition);
            turret.setPowerResult(power);
        }
    }
}
