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
        FIRST_PATH (FIRST_MOVE, 0.5),
        SECOND_PATH (SECOND_MOVE, 4.0),
        THIRD_PATH (THIRD_MOVE, 3.5),
        FOURTH_PATH (FOURTH_MOVE, 0.5),
        FIFTH_PATH (FIFTH_MOVE, 2.0),
        SIXTH_PATH (SIXTH_MOVE, 0.5),
        SEVENTH_PATH (SEVENTH_MOVE, 5),
        EIGHTH_PATH (EIGHTH_MOVE, 5),
        NINTH_PATH (NINTH_MOVE, 5),
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
                            new Pose(0, 0, -0.7068),
                            new Pose(-14.711, -7.6111, -0.7068)
                    }
            ))
            .build();

    static PathContainer SECOND_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.05)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-14.711, -7.6111, Math.toRadians(0)),
                            new Pose(-9.37, -30, Math.toRadians(0))
                    }
            ))
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-9.37, -30, -0.23),
                            new Pose(-66.21, -7.83, -0.23)
                    }
            ))
            .build();

    static PathContainer THIRD_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-66.21, -7.83, -0.23),
                            new Pose(-58.21, 22, -0.23)
                    }
            ))
            .build();

    static PathContainer FOURTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.1)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-58.21, 19.41, -0.23),
                            new Pose(-66.21, -20, -0.23),
                    }
            ))
            .build();

    static PathContainer FIFTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.1)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-66.21, -20, -0.23),
                            new Pose(-14.711, -20, -0.7068),
                    }
            ))
            .build();

    static PathContainer SIXTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.1)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-14.711, -20, -0.7068),
                            new Pose(-14.711, -7.6111, -0.7068),
                    }
            ))
            .build();

    static PathContainer SEVENTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.1)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-14.711, -7.6111, -0.23),
                            new Pose(-44, -20, -0.147),
                    }
            ))
            .build();

    static PathContainer EIGHTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.1)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-44, -20, -0.147),
                            new Pose(-39.7, 15.0, -0.151),
                    }
            ))
            .build();

    static PathContainer NINTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.1)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-39.7, 15.0, -0.847),
                            new Pose(-15.5, -4.08, -0.847),
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
                .setLookAhead(1.0)
                .setSpeed(0.6)
                .setHeadingPID(new PID(PestoFTCConfig.HEADING_KP, 0, 0))
                .setEndpointPID(new PID(PestoFTCConfig.ENDPOINT_KP, 0, 0))
                .build();

        return pathFollower;
    }

    public static PathFollower getPathFollower(PathState state, double driveSpeed) {
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
                .setLookAhead(2.0)
                .setSpeed(driveSpeed)
                .setHeadingPID(new PID(PestoFTCConfig.HEADING_KP, 0, 0))
                .setEndpointPID(new PID(PestoFTCConfig.ENDPOINT_KP, 0, 0))
                .build();

        return pathFollower;
    }
}
