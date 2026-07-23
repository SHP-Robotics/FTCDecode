package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.algorithms.Interpolator;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.ConfigInterface;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoConfig;

import java.util.Collections;
import java.util.List;

//@Config
@PestoConfig()
public class PestoFTCConfig implements ConfigInterface {
    public static boolean initialized = false; // don't mess with this :O
    public static boolean initializePinpoint = true;

    public static Follower follower;

    // BLOCKER
    public static double BLOCKER_BLOCK = 0.34;
    public static double BLOCKER_OUTTAKE = 0.54;

    // INDEXER
    public static double INDEXER_IN = 0.20;
    public static double INDEXER_OUTISH = 0.54;
    public static double INDEXER_OUT = 0.589;

    public static double[] shooter_close = new double[]{51.3, -0.58};
    public static double[] shooter_mid = new double[]{92.5, -0.71};
    public static double[] shooter_far = new double[]{132.0, -0.87};

    public static Pair<Double, Double>[] interpolatorShooterData = new Pair[]{
            new Pair<Double, Double>(shooter_close[0], shooter_close[1]),
            new Pair<Double, Double>(shooter_mid[0], shooter_mid[1]),
            new Pair<Double, Double>(shooter_far[0], shooter_far[1]),
    };

    public static Interpolator interpolatorShooter = new Interpolator(interpolatorShooterData);

    public static double[] hood_close = new double[]{51.3, 0.70};
    public static double[] hood_mid = new double[]{92.5, 0.70};
    public static double[] hood_far = new double[]{132.0, 0.70};

    public static Pair<Double, Double>[] interpolatorHoodData = new Pair[]{
            new Pair<Double, Double>(hood_close[0], hood_close[1]),
            new Pair<Double, Double>(hood_mid[0], hood_mid[1]),
            new Pair<Double, Double>(hood_far[0], hood_far[1]),
    };

    public static Interpolator interpolatorHood = new Interpolator(interpolatorHoodData);

    public static void recalculate_interpolators() {
        interpolatorShooterData = new Pair[]{
                new Pair<Double, Double>(shooter_close[0], shooter_close[1]),
                new Pair<Double, Double>(shooter_mid[0], shooter_mid[1]),
                new Pair<Double, Double>(shooter_far[0], shooter_far[1]),
        };

        interpolatorShooter = new Interpolator(interpolatorShooterData);

        interpolatorHoodData = new Pair[]{
                new Pair<Double, Double>(hood_mid[0], hood_mid[1]),
                new Pair<Double, Double>(hood_mid[0], hood_mid[1]),
                new Pair<Double, Double>(hood_far[0], hood_far[1]),
        };

        interpolatorHood = new Interpolator(interpolatorHoodData);
    }

    // TURRET
    public static List<Integer> BLUE_TAGS = Collections.singletonList(20);
    public static List<Integer> RED_TAGS = Collections.singletonList(24);

    // 10 degrees switch
    public static double TURRET_BEARING_SWITCH = 3.0;

    public static double TURRET_KP_SWITCH = 5.0 * 310 / 45;
    public static double TURRET_KP_PRIMARY = 0.005;
    public static double TURRET_KP_SECONDARY = 0.010;

    public static void initialize(HardwareMap hardwareMap) {
        MotorCortex.initialize(hardwareMap);

        MecanumController mecanumController = new MecanumController(
                MotorCortex.getMotor("frontL"),
                MotorCortex.getMotor("frontR"),
                MotorCortex.getMotor("backL"),
                MotorCortex.getMotor("backR")
        );

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        });

        mecanumController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (initializePinpoint) {
            follower = Constants.createFollower(hardwareMap);

            DeterministicTracker tracker = new DeterministicTracker() {
                @Override
                public Pose getRobotVelocity() {
                    Vector velocity = follower.getVelocity();

                    return new Pose(
                            velocity.getXComponent(),
                            velocity.getYComponent(),
                            velocity.getTheta()
                    );
                }

                @Override
                public Pose getDeltaPosition() {
                    return null;
                }

                @Override
                public Pose getCentripetalForce() {
                    return new Pose(0, 0, 0);
                }

                @Override
                public void reset() {
                    follower.setPose(new com.pedropathing.geometry.Pose(0, 0));
                }

                @Override
                public void reset(double v) {
                    follower.setPose(new com.pedropathing.geometry.Pose(0, 0, v));
                }

                @Override
                public void reset(Pose pose) {
                    follower.setPose(new com.pedropathing.geometry.Pose(pose.getY(), -pose.getX(), pose.getHeadingRadians()));
                }

                @Override
                public void update() {
                    follower.update();
                }

                @Override
                public Pose getCurrentPosition() {
                    com.pedropathing.geometry.Pose pose = follower.getPose();

                    return new Pose(
                            -pose.getY(), pose.getX(), pose.getHeading()
                    );
                }
            };

            TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);
            teleOpController.useTrackerIMU(tracker);

            teleOpController.setSpeedController(gamepad -> 1.0);

            FrontalLobe.teleOpController = teleOpController;
            FrontalLobe.tracker = tracker;
        }

        FrontalLobe.driveController = mecanumController;
    }
}
