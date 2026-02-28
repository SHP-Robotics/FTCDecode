package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.AUTO;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.CUSTOM_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.MANUAL;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.RED;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.TurretState.STRAIGHT_SOLID;
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
    private AprilTagPoseFtc lastAprilTag = null;
    private long lastAprilTagTimer = 0;
    private double visionOffset = 0.0;

    private TurretState state;

    private double customPosition;

    public enum TurretState {
        LEFT,
        STRAIGHT,
        STRAIGHT_SOLID,
        RIGHT,
        RED,
        BLUE,

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
//        assert state == MANUAL;
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

    public AprilTagPoseFtc getAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null)
                continue;

            // TODO: INSERT MAGIC HERE
            // detection.ftcPose.yaw

            if (!acceptedTags.contains(detection.id))
                continue;

            lastAprilTag = detection.ftcPose;
            lastAprilTagTimer = System.nanoTime();
            return lastAprilTag;
        }

        lastAprilTag = null;
        return null;
    }

    public Double getAprilTagBearing() {
        if (lastAprilTag == null)
            return 0.0;

        return lastAprilTag.bearing;
    }

    public Double getAprilTagDistance() {
        if (lastAprilTag == null)
            return 0.0;

        return lastAprilTag.y;
    }

    public Double getAprilTagYaw() {
        if (lastAprilTag == null)
            return 0.0;

        return -lastAprilTag.yaw;
    }

//    public Double getTargetBearing() {
//        getAprilTag();
//
//        if (lastAprilTag == null)
//            return null;
//
//        Double bearing = getAprilTagBearing();
//        Double distance = getAprilTagDistance();
//
//        assert bearing != null;
//        assert distance != null;
//
//        double heading = -bearing - getDegrees();
//        heading = 90 - heading + RobotPose.robotPosition.getHeadingRadians();
//
//        double aprilTagX = getAprilTagDistance() * Math.cos(Math.toRadians(heading));
//        double aprilTagY = getAprilTagDistance() * Math.sin(Math.toRadians(heading));
//
//        double targetHeading = Math.toDegrees(Math.atan2(aprilTagY + (18 / Math.sqrt(2)), aprilTagX + (18 / Math.sqrt(2))));
//
//        return targetHeading - 90;
//    }

    public double getDegrees() {
        return this.getPosition() * 45 / 310;
    }

    public double getTargetPosition() {
        double targetPosition = 0;

        if (this.state == RED)
            targetPosition = 930;

        if (this.state == BLUE)
            targetPosition = 1550;

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
//        double power = cameraPIDController.getOutput(turretDegrees - RobotPose.robotPosition.getHeadingRadians(), bearing);

        // Approx 15 degrees left
        if (this.getPosition() < getTargetPosition() - 620)
            power = Math.max(power, 0);

        // Approx 15 degrees right
        if (this.getPosition() > getTargetPosition() + 1860)
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
            // update
            getAprilTag();
//            Double bearing = getTargetBearing();
            Double bearing = getAprilTagBearing();

            if (bearing != null) {
                double turretDegrees = this.getDegrees();
                double power = cameraPIDController.getOutput(turretDegrees + visionOffset, turretDegrees + bearing);

                turret.setPowerResult(power);
            } else
                turret.setPowerResult(0.0);

            return;
        }

        if (this.state == STRAIGHT_SOLID) {
            // Position Processing Code

            double targetPosition = getTargetPosition();

            if (useSecondary()) {
                double power = positionPIDControllerSecondary.getOutput(turret.getCurrentPosition(), targetPosition);
                turret.setPowerResult(power);
            } else {
                double power = positionPIDControllerPrimary.getOutput(turret.getCurrentPosition(), targetPosition);
                turret.setPowerResult(power);
            }

            return;
        }

        // Camera Processing Code

        getAprilTag();
        if (lastAprilTag != null) {
            Double bearing = getAprilTagBearing(); //getTargetBearing();
            this.setBearing(bearing);
            return;
        }

        // So far zone can recognize
        if ((System.nanoTime() - lastAprilTagTimer) / 1E9 < 0.5) {
            turret.setPowerResult(0.0);
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
