package org.firstinspires.ftc.teamcode.autonomous;

import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class BlueFarRealPaths {
    public enum PathState {
        FIRST_PATH (FIRST_MOVE, 2),
        SECOND_PATH (SECOND_MOVE, 4),
        THIRD_PATH (THIRD_MOVE, 4),
        FOURTH_PATH (FOURTH_MOVE, 4),
        FIFTH_PATH (FIFTH_MOVE, 4),
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
                            new Pose(0, 0, 0.52),
                            new Pose(0, 6, 0.52)
                    }
            ))
            .build();

    static PathContainer SECOND_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.05)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(0.0, 6, 2.13),
                            new Pose(0.0, 17, 2.13)
                    }
            ))
            .build();

    static PathContainer THIRD_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(0, 17, 2.13),
                            new Pose(-44, 17, 2.13)
                    }
            ))
            .build();

    static PathContainer FOURTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-44, 17, 2.13),
                            new Pose(-44, 4.2, 2.13)
                    }
            ))
            .build();

    static PathContainer FIFTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.1)
            .addCurve(new BezierCurve(
                    new Pose[]{
                            new Pose(-20, 30, 0.52),
                            new Pose(0, 3, 0.52),
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
