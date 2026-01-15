package org.firstinspires.ftc.teamcode.autonomous;

import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class BlueFarPaths {
    public enum PathState {
        FIRST_PATH (FIRST_MOVE, 2.0),
        SECOND_PATH (SECOND_MOVE, 5.0),
        THIRD_PATH (THIRD_MOVE, 5.0),
        FOURTH_PATH (FOURTH_MOVE, 5.0),
        DONE (null, Double.POSITIVE_INFINITY);

        PathState(PathContainer pathContainer, double timer) {
            this.pathContainer = pathContainer;
            this.timer = timer;
        }

        PathContainer pathContainer;
        double timer;

        PathContainer getPath() {
            return this.pathContainer;
        }

        double getTimer() {
            return this.timer;
        }
    }

    static PathContainer FIRST_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(0, 0, Math.toRadians(0)),
                            new Pose(-8.5, -0.3, Math.toRadians(-45))
                    }
            ))
            .build();

    static PathContainer SECOND_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-8.5, -0.3, Math.toRadians(0)),
                            new Pose(-8.5, -0.3, Math.toRadians(0))
                    }
            ))
            .build();

    static PathContainer THIRD_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-8.5, -0.3, Math.toRadians(0)),
                            new Pose(-67.45, -30, Math.toRadians(0))
                    }
            ))
            .build();

    static PathContainer FOURTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-67.45, -30, Math.toRadians(0)),
                            new Pose(-64, 15, Math.toRadians(0))
                    }
            ))
            .build();

    public static PathFollower getPathFollower(PathState state) {
        // TODO: create secondary
        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                FrontalLobe.driveController,
                FrontalLobe.tracker,
                state.getPath(),
                0.2,
                0.0,
                0.2
        )
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setLookAhead(1.5)
                .setSpeed(0.4)
                .setHeadingPID(new PID(PestoFTCConfig.HEADING_KP, 0, 0))
                .setEndpointPID(new PID(PestoFTCConfig.ENDPOINT_KP, 0, 0))
                .build();

        return pathFollower;
    }
}
