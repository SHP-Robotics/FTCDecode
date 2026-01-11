package org.firstinspires.ftc.teamcode.autonomous;

import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class TestAutoPaths {
    public enum PathState {
        FIRST_PATH (FIRST_MOVE, 2.25),
        SECOND_PATH (SECOND_MOVE, 1.5),
        THIRD_PATH (THIRD_MOVE, 2.0),
        FOURTH_PATH (FOURTH_MOVE, 2.25),
        FIFTH_PATH (FIFTH_MOVE, 2.3),
        SIXTH_PATH(SIXTH_MOVE, 2.5),
        SEVENTH_PATH(SEVENTH_MOVE,1),
        EIGHTH_PATH(EIGHTH_MOVE, 2.3),
        DONE (null, Double.POSITIVE_INFINITY);

        PathState(PathContainer path, double timer) {
            this.path = path;
            this.timer = timer;
        }

        final PathContainer path;
        final double timer;

        PathContainer getPath() {
            return path;
        }

        double getTimer() {
            return timer;
        }
    }

    public static PathContainer FIRST_MOVE = new PathContainer.PathContainerBuilder()
            .setStartPosition(new Pose(0, 0))
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //goes to shoot preloads
                            new Pose(0, 0),
                            new Pose(-15, 0)
                    }
            ), new ParametricHeading(v -> 0.0))
            .build();

    public static PathContainer SECOND_MOVE = new PathContainer.PathContainerBuilder()
            .setStartPosition(new Pose(-15, 0))
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //goes to intake spike
                            new Pose(-15, 0),
                            new Pose(-29, 13)
                    }
            ), new ParametricHeading(v -> 0.0))
            .build();

    public static PathContainer THIRD_MOVE = new PathContainer.PathContainerBuilder()
            .setStartPosition(new Pose(-29, 13))
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //does the intake
                            new Pose(-29, 13),
                            new Pose(-29, 50)
                    }
            ), new ParametricHeading(v -> 0.0))
            .build();

    public static PathContainer FOURTH_MOVE = new PathContainer.PathContainerBuilder()
            .setStartPosition(new Pose(-29, 50))
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ // goes back to shoot
                            new Pose(-29, 50),
                            new Pose(-15, 0)
                    }
            ), new ParametricHeading(v -> 0.0))
            .build();

    public static PathContainer FIFTH_MOVE = new PathContainer.PathContainerBuilder()
            .setStartPosition(new Pose(-29, 50))
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //goes to human player
                            new Pose(-29, 50),
                            new Pose(-15, 0)
                    }
            ), new ParametricHeading(v -> 0.0))
            .build();

    public static PathContainer SIXTH_MOVE = new PathContainer.PathContainerBuilder()
            .setStartPosition(new Pose(-29, 50))
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //intakes human player
                            new Pose(-29, 50),
                            new Pose(-15, 0)
                    }
            ), new ParametricHeading(v -> 0.0))
            .build();

    public static PathContainer SEVENTH_MOVE = new PathContainer.PathContainerBuilder()
            .setStartPosition(new Pose(-29, 50))
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //back to shoot
                            new Pose(-29, 50),
                            new Pose(-15, 0)

                    }
            ), new ParametricHeading(v -> 0.0))
            .build();
    public static PathContainer EIGHTH_MOVE = new PathContainer.PathContainerBuilder()
            .setStartPosition(new Pose(-29, 50))
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //leave
                            new Pose(-29, 50),
                            new Pose(-15, 0)
                    }
            ), new ParametricHeading(v -> 0.0))
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
