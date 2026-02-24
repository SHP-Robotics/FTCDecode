package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class AutoPathsRedClose {
    public enum PathState {
        FIRST_PATH (FIRST_MOVE, 10),
        SECOND_PATH (SECOND_MOVE, 10),
        THIRD_PATH (THIRD_MOVE, 10),
        FOURTH_PATH (FOURTH_MOVE, 10),
        FIFTH_PATH (FIFTH_MOVE, 10),
        SIXTH_PATH(SIXTH_MOVE, 10),
        SEVENTH_PATH(SEVENTH_MOVE,10),
        EIGHTH_PATH(EIGHTH_MOVE, 10);

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
                    new Pose(0, 15)
            )
    );

    public static Path SECOND_MOVE = new Path(
            new BezierLine(
                    new Pose(0, 15),
                    new Pose(50, 15)
            )
    );

    public static Path THIRD_MOVE = new Path(
            new BezierLine(
                    new Pose(50, 15),
                    new Pose(50, 4)
            )
    );

    public static Path FOURTH_MOVE = new Path(
            new BezierLine(
                    new Pose(50, 4),
                    new Pose(0, 15)
            )
    );

    public static Path FIFTH_MOVE = new Path(
            new BezierLine(
                    new Pose(0, 15),
                    new Pose(45, 1)
            )
    );

    public static Path SIXTH_MOVE = new Path(
            new BezierLine(
                    new Pose(45, 1),
                    new Pose(45, 3)
            )
    );

    public static Path SEVENTH_MOVE = new Path(
            new BezierLine(
                    new Pose(45, 3),
                    new Pose(0, 15)
            )
    );

    public static Path EIGHTH_MOVE = new Path(
            new BezierLine(
                    new Pose(0, 15),
                    new Pose(30, 4)
            )
    );

    public static void initializePaths() {
        FIRST_MOVE.setConstantHeadingInterpolation(0.0);
        SECOND_MOVE.setLinearHeadingInterpolation(0, Math.toRadians(-90));
        THIRD_MOVE.setConstantHeadingInterpolation(Math.toRadians(-90));
        FOURTH_MOVE.setLinearHeadingInterpolation(Math.toRadians(-90), 0.0);
        FIFTH_MOVE.setConstantHeadingInterpolation(0.0);
        SIXTH_MOVE.setConstantHeadingInterpolation(0.0);
        SEVENTH_MOVE.setConstantHeadingInterpolation(0.0);
        EIGHTH_MOVE.setConstantHeadingInterpolation(0.0);
    }
}