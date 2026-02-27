package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class AutoPathsRedClose {
    public enum PathState {
        SHOOT_PATH (SHOOT, 1.5),
        FIRST_PATH (FIRST_MOVE, 1.5),
        SECOND_PATH (SECOND_MOVE, 1.5),
        THIRD_PATH (THIRD_MOVE, 2),
        FOURTH_PATH (FOURTH_MOVE, 2.5),
        FIFTH_PATH (FIFTH_MOVE, 2),
        SIXTH_PATH (SIXTH_MOVE, 2.5),
        SEVENTH_PATH (SEVENTH_MOVE, 2),
        EIGHTH_PATH (EIGHTH_MOVE, 2.5);

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

    public static Path SHOOT = new Path(
            new BezierLine(
                    new Pose(0, 0),
                    new Pose(-2, 15)
            )
    );

    public static Path FIRST_MOVE = new Path(
            new BezierLine(
                    new Pose(-2, 15),
                    new Pose(13, 29)
            )
    );

    public static Path SECOND_MOVE = new Path(
            new BezierLine(
                    new Pose(13, 29),
                    new Pose(41.8, 29)
            )
    );

    public static Path THIRD_MOVE = new Path(
            new BezierLine(
                    new Pose(45.8, 29),
                    new Pose(-2, 15)
            )
    );

    public static Path FOURTH_MOVE = new Path(
            new BezierLine(
                    new Pose(-2, 15),
                    new Pose(43.3, 3)
            )
    );

    public static Path FIFTH_MOVE = new Path(
            new BezierLine(
                    new Pose(43.3, 3),
                    new Pose(-2, 15)
            )
    );

    public static Path SIXTH_MOVE = new Path(
            new BezierLine(
                    new Pose(-2, 15),
                    new Pose(43.3, 15)
            )
    );

    public static Path SEVENTH_MOVE = new Path(
            new BezierLine(
                    new Pose(43.3, 15),
                    new Pose(-2, 15)
            )
    );

    public static Path EIGHTH_MOVE = new Path(
            new BezierLine(
                    new Pose(-2, 15),
                    new Pose(0.0, 27)
            )
    );

    public static void initializePaths() {
        SHOOT.setConstantHeadingInterpolation(0.0);
        FIRST_MOVE.setConstantHeadingInterpolation(0.0);
        SECOND_MOVE.setConstantHeadingInterpolation(0.0);
        THIRD_MOVE.setConstantHeadingInterpolation(0.0);
        FOURTH_MOVE.setConstantHeadingInterpolation(0.0);
        FIFTH_MOVE.setConstantHeadingInterpolation(0.0);
        SIXTH_MOVE.setConstantHeadingInterpolation(0.0);
        SEVENTH_MOVE.setConstantHeadingInterpolation(0.0);
        EIGHTH_MOVE.setConstantHeadingInterpolation(0.0);
    }
}