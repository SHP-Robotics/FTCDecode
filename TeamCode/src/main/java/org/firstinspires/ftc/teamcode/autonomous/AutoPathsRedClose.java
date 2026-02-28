package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class AutoPathsRedClose {
    public enum PathState {
        FIRST_PATH (FIRST_MOVE, 5),
        SECOND_PATH (SECOND_MOVE, 5),
        THIRD_PATH (THIRD_MOVE, 5),
        FOURTH_PATH (FOURTH_MOVE, 5);

        PathState(Path path, double timer) {
            this.path = path;
            this.timer = timer;
        }

        final Path path;
        final double timer;

        Path getPath() {
            return path;
        }

        double getTimer() {
            return timer;
        }
    }

    // -2, 15

    public static Path FIRST_MOVE = new Path(
            new BezierLine(
                    new Pose(0, 0),
                    new Pose(0, -29)
            )
    );

    public static Path SECOND_MOVE = new Path(
            new BezierLine(
                    new Pose(0, -29),
                    new Pose(0, -76)
            )
    );

    public static Path THIRD_MOVE = new Path(
            new BezierLine(
                    new Pose(0, -76),
                    new Pose(18, -76)
            )
    );

    public static Path FOURTH_MOVE = new Path(
            new BezierCurve(
                    new Pose(18, -76),
                    new Pose(0, -76),
                    new Pose(0, -29)
            )
    );

    public static void initializePaths() {
        FIRST_MOVE.setConstantHeadingInterpolation(0.0);
        SECOND_MOVE.setConstantHeadingInterpolation(0.0);
        THIRD_MOVE.setConstantHeadingInterpolation(0.0);
        FOURTH_MOVE.setConstantHeadingInterpolation(0.0);
    }
}