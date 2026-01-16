package org.firstinspires.ftc.teamcode.autonomous;

import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class TestAutoPathsRed {
    public enum PathState {
        FIRST_PATH (FIRST_MOVE, 0.55),
        SECOND_PATH (SECOND_MOVE, 0.9),
        THIRD_PATH (THIRD_MOVE, 1.2),
        FOURTH_PATH (FOURTH_MOVE, 1.8),
        FIFTH_PATH (FIFTH_MOVE, 2.0),
        SIXTH_PATH(SIXTH_MOVE, 2.5),
        SEVENTH_PATH(SEVENTH_MOVE,2.5),
        EIGHTH_PATH(EIGHTH_MOVE, 1.8),
        NINETH_PATH(NINETH_MOVE, 5),
        TENTH_PATH(TENTH_MOVE, 3),
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
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //goes to shoot preloads
                            new Pose(0, 0, Math.toRadians(0.0)),
                            new Pose(-15, 0, Math.toRadians(0.0))
                    }
            ))
            .build();

    public static PathContainer SECOND_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //goes to intake spike
                            new Pose(-15, 0, Math.toRadians(0.0)),
                            new Pose(-29, 13, Math.toRadians(0.0))
                    }
            ))
            .build();

    public static PathContainer THIRD_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //does the intake
                            new Pose(-29, 13, Math.toRadians(0.0)),
                            new Pose(-29, 50, Math.toRadians(0.0))
                    }
            ))
            .build();

    public static PathContainer FOURTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ // goes back to shoot
                            new Pose(-29, 50, Math.toRadians(0.0)),
                            new Pose(-15, 0, Math.toRadians(0.0))
                    }
            ))
            .build();

    public static PathContainer FIFTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //goes to human player
                            new Pose(-29, 50, Math.toRadians(-90)),
                            new Pose(-18, 50, Math.toRadians(-90))
                    }
            ))
            .build();

    public static PathContainer SIXTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //intakes human player
                            new Pose(-18, 50, Math.toRadians(-90)),
                            new Pose(0, 50, Math.toRadians(-90))
                    }
            ))
            .build();

    public static PathContainer SEVENTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //back to shoot
                            new Pose(-0, 50, Math.toRadians(0.0)),
                            new Pose(-15, 0, Math.toRadians(0.0))

                    }
            ))
            .build();
    public static PathContainer EIGHTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //goto pickup
                            new Pose(-15, 0, Math.toRadians(0.)),
                            new Pose(5, 45 , Math.toRadians(0.0))
                    }
            ))
            .build();
    public static PathContainer NINETH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //back to shoot
                            new Pose(5, 45, Math.toRadians(0.0)),
                            new Pose(-15, 0 , Math.toRadians(0.0))
                    }
            ))
            .build();
    public static PathContainer TENTH_MOVE = new PathContainer.PathContainerBuilder()
            .setIncrement(0.01)
            .addCurve(new BezierCurve(
                    new Pose[]{ //leave
                            new Pose(-15, 0, Math.toRadians(0.0)),
                            new Pose(-15, 10 , Math.toRadians(0.0))
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
        0.05,
                0.2
        )
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setLookAhead(1.5)
                .setSpeed(0.8)
                .setHeadingPID(new PID(PestoFTCConfig.HEADING_KP, 0, 0))
                .setEndpointPID(new PID(PestoFTCConfig.ENDPOINT_KP, 0, 0))
                .build();

        return pathFollower;
    }
}
