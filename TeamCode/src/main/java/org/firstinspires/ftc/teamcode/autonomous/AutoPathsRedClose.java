package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class AutoPathsRedClose {
    public enum PathState {
        FIRST_PATH (FIRST_MOVE, 2),
        SECOND_PATH (SECOND_MOVE, 1.5),
        THIRD_PATH (THIRD_MOVE, 3),
        FOURTH_PATH (FOURTH_MOVE, 2.5),
        FIFTH_PATH (FIFTH_MOVE, 2.5);

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

    public static Path FIRST_MOVE = new Path(
            new BezierLine(
                    new Pose(0, 0),
                    new Pose(14.8, 27)
            )
    );

    public static Path SECOND_MOVE = new Path(
            new BezierLine(
                    new Pose(14.8, 27),
                    new Pose(41.8, 27)
            )
    );

    public static Path THIRD_MOVE = new Path(
            new BezierLine(
                    new Pose(45.8, 27),
                    new Pose(0, 3)
            )
    );

    public static Path FOURTH_MOVE = new Path(
            new BezierLine(
                    new Pose(0.0, 3),
                    new Pose(43.3, 3)
            )
    );

    public static Path FIFTH_MOVE = new Path(
            new BezierLine(
                    new Pose(43.3, 3),
                    new Pose(0.0, 3)
            )
    );

    public static void initializePaths() {
        FIRST_MOVE.setConstantHeadingInterpolation(0.0);
        SECOND_MOVE.setConstantHeadingInterpolation(0.0);
        THIRD_MOVE.setConstantHeadingInterpolation(0.0);
        FOURTH_MOVE.setConstantHeadingInterpolation(0.0);
        FIFTH_MOVE.setConstantHeadingInterpolation(0.0);
    }
}