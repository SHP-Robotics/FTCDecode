package org.firstinspires.ftc.teamcode.teleops;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/** Turns to zero tag roll with an upward-tilted camera. Positive drive turn is clockwise. */
final class AprilTagAlignmentController {
    private static final double TURN_GAIN = 0.03;
    private boolean aligning;
    private int targetId = -1;
    private double rollErrorDeg = Double.NaN;
    private String status = "Idle - B: level tag roll";

    boolean isAligning() {
        return aligning;
    }

    int getTargetId() {
        return targetId;
    }

    String getStatus() {
        return status;
    }

    double getRollErrorDeg() {
        return rollErrorDeg;
    }

    Double update(List<AprilTagDetection> detections) {
        AprilTagDetection target = selectTarget(detections, targetId);
        if (target == null)
            return null;

        // Wrap signed roll into [0, 360), preserving the angle instead of clipping it.
        rollErrorDeg = (target.ftcPose.roll + 360.0) % 360.0;
        targetId = target.id;

        return Math.max(-0.4, Math.min(0.4, (180 - rollErrorDeg) * TURN_GAIN));
    }

    static AprilTagDetection selectTarget(List<AprilTagDetection> detections, int id) {
        AprilTagDetection best = null;
        for (AprilTagDetection detection : detections) {
            if ((id >= 0 && detection.id != id) || detection.metadata == null
                    || detection.ftcPose == null
                    || !Double.isFinite(detection.ftcPose.roll)
                    || !Double.isFinite(detection.ftcPose.bearing)
                    || !Double.isFinite(detection.ftcPose.range) || detection.ftcPose.range <= 0) {
                continue;
            }
            if (best == null || Math.abs(detection.ftcPose.bearing) < Math.abs(best.ftcPose.bearing)
                    || (Math.abs(detection.ftcPose.bearing) == Math.abs(best.ftcPose.bearing)
                    && detection.ftcPose.range < best.ftcPose.range)) {
                best = detection;
            }
        }
        return best;
    }
}
