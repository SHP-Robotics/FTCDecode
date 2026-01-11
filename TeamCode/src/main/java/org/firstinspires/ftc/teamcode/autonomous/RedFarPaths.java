package org.firstinspires.ftc.teamcode.autonomous;

import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class RedFarPaths {
    public enum PathState {
        FIRST_PATH (FIRST_MOVE, 5.0),
        SECOND_PATH (FIRST_ROTATE, 5.0),
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
                            new Pose(0, 0),
                            new Pose(0, 10)
                    }
            ), new ParametricHeading(v -> 0.0))
            .build();

    static PathContainer FIRST_ROTATE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .setStartPosition(new Pose(0, 10))
            .addCurve(new ParametricHeading(v -> -Math.PI*v/3))
            .build();

    public static PathFollower getPathFollower(PathState state) {
        // TODO: create secondary
        PathFollower pathFollower = new PathFollower.PathFollowerBuilder(
                FrontalLobe.driveController,
                FrontalLobe.tracker,
                state.getPath()
        )
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setLookAhead(1.5)
                .setSpeed(0.75)
                .setHeadingPID(new PID(0.3, 0, 0))
                .setEndpointPID(new PID(0.001, 0, 0))
                .build();

        return pathFollower;
    }
}
