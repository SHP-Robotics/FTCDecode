package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.AUTO;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.CUSTOM_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.RIGHT;
import static java.util.Arrays.asList;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
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
    private double lastDistance = 0.0;
    private double visionOffset = 0.0;

    private TurretState state;

    private double customPosition;

    public enum TurretState {
        LEFT,
        STRAIGHT,
        RIGHT,

        CUSTOM_POSITION,
        MANUAL,
        AUTO
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

    public void setState(TurretState state) {
        this.state = state;
    }

    public void setPower(double power) {
        assert state == MANUAL;
        turret.setPowerResult(power);
    }

    public void setPosition(double position) {
        this.customPosition = position;
        this.state = CUSTOM_POSITION;
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

    public Double getAprilTagBearing() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null)
                continue;

            if (!acceptedTags.contains(detection.id))
                continue;

            lastDistance = detection.ftcPose.y;
            return detection.ftcPose.bearing;
        }

        return null;
    }

    public double getAprilTagDistance() {
        return this.lastDistance;
    }

    public double getDegrees() {
        return this.getPosition() * 45 / 310;
    }

    public double getTargetPosition() {
        double targetPosition = 0;

        if (this.state == LEFT)
            targetPosition = PestoFTCConfig.TURRET_LEFT;

        if (this.state == RIGHT)
            targetPosition = PestoFTCConfig.TURRET_RIGHT;

        if (this.state == CUSTOM_POSITION)
            targetPosition = customPosition;

        return targetPosition;
    }

    public void setBearing(double bearing) {
        double turretDegrees = this.getDegrees();
        double power = cameraPIDController.getOutput(turretDegrees, turretDegrees + bearing);

        // Approx 15 degrees left
        if (this.getPosition() < getTargetPosition() - 620)
            power = Math.max(power, 0);

        // Approx 15 degrees right
        if (this.getPosition() > getTargetPosition() + 620)
            power = Math.min(power, 0);

        turret.setPowerResult(power);
    }

    public boolean useSecondary() {
        double targetPosition = getTargetPosition();
        return Math.abs(turret.getCurrentPosition() - targetPosition) < PestoFTCConfig.TURRET_KP_SWITCH;
    }

    public void update() {
        if (this.state == MANUAL)
            return;

        if (this.state == AUTO) {
            Double bearing = getAprilTagBearing();

            if (bearing != null) {
                double turretDegrees = this.getDegrees();
                double power = cameraPIDController.getOutput(turretDegrees + visionOffset, turretDegrees + bearing);

                turret.setPowerResult(power);
            } else
                turret.setPowerResult(0.0);

            return;
        }

        // Camera Processing Code

        Double bearing = getAprilTagBearing();

        if (bearing != null) {
            this.setBearing(bearing);
            return;
        }

        // Position Processing Code

        double targetPosition = getTargetPosition();

        if (useSecondary()) {
            double power = positionPIDControllerSecondary.getOutput(turret.getCurrentPosition(), targetPosition);
            turret.setPowerResult(power);
        } else {
            double power = positionPIDControllerPrimary.getOutput(turret.getCurrentPosition(), targetPosition);
            turret.setPowerResult(power);
        }
    }
}
