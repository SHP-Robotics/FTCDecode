package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@PestoConfig()
public class PestoFTCConfig implements ConfigInterface {
    public static boolean initialized = false; // don't mess with this :O
    public static boolean initializePinpoint = true;

    public static Follower follower;

//    public static double mass = 23.7; // lbs
//    public static double MAX_FORCE = 2170;
//    public static double DECELERATION = 100;
//    public static double ENDPOINT_KP = 0.3;
//    public static double HEADING_KP = 3.5;
//
//    // ODOMETRY
//    public static String leftName = "backL";
//    public static String centerName = "backR";
//    public static String rightName = "frontR";
//
//    public static DcMotorSimple.Direction leftDirection = DcMotorSimple.Direction.FORWARD;
//    public static DcMotorSimple.Direction centerDirection = DcMotorSimple.Direction.REVERSE;
//    public static DcMotorSimple.Direction rightDirection = DcMotorSimple.Direction.FORWARD;
//
//    public static double ODOMETRY_TICKS_PER_INCH = 505.3169;
//    public static double FORWARD_OFFSET = -1.9;
//    public static double ODOMETRY_WIDTH = 4.2768;

    public static double STATIC_DRIVE = 0.22;

    // DROPDOWN
//    public static double DROPDOWN_DRIVE = 0.4;
//    public static double DROPDOWN_INTAKE = 0.25;
//    public static double DROPDOWN_PUSH = 0.35;

    // BLOCKER
    public static double BLOCKER_BLOCK = 0.34;
    public static double BLOCKER_OUTTAKE = 0.0;

    // INDEXER
    public static double INDEXER_IN = 0.20;
    public static double INDEXER_OUTISH = 0.52;
    public static double INDEXER_OUT = 0.589;

    // HOOD
    public static double HOOD_CLOSE = 0.72;
    public static double HOOD_MID = 0.45;
    public static double HOOD_FAR = 0.37;

    // SHOOTER
    public static double SHOOTER_KP = 0.3;
    public static double SHOOTER_KD = 0.05;

    public static double[] shooter040 = new double[]{040.0, -0.51};
    public static double[] shooter050 = new double[]{050.0, -0.52};
    public static double[] shooter060 = new double[]{060.0, -0.55};
    public static double[] shooter070 = new double[]{070.0, -0.55};
    public static double[] shooter080 = new double[]{080.0, -0.63};
    public static double[] shooter090 = new double[]{090.0, -0.61};
    public static double[] shooter100 = new double[]{100.0, -0.95};
    public static double[] shooter110 = new double[]{110.0, -1.00};

    public static Pair<Double, Double>[] interpolatorShooterData = new Pair[]{
            new Pair<Double, Double>(shooter040[0], shooter040[1]),
            new Pair<Double, Double>(shooter050[0], shooter050[1]),
            new Pair<Double, Double>(shooter060[0], shooter060[1]),
            new Pair<Double, Double>(shooter070[0], shooter070[1]),
            new Pair<Double, Double>(shooter080[0], shooter080[1]),
            new Pair<Double, Double>(shooter090[0], shooter090[1]),
            new Pair<Double, Double>(shooter100[0], shooter100[1]),
            new Pair<Double, Double>(shooter110[0], shooter110[1]),
    };

    public static Interpolator interpolatorShooter = new Interpolator(interpolatorShooterData);

    public static double[] hood040 = new double[]{040.0, 0.45};
    public static double[] hood050 = new double[]{050.0, 0.70};
    public static double[] hood060 = new double[]{060.0, 0.80};
    public static double[] hood070 = new double[]{070.0, 0.85};
    public static double[] hood080 = new double[]{080.0, 0.85};
    public static double[] hood090 = new double[]{090.0, 0.77};
    public static double[] hood100 = new double[]{100.0, 1.00};
    public static double[] hood110 = new double[]{110.0, 1.00};

    public static Pair<Double, Double>[] interpolatorHoodData = new Pair[]{
            new Pair<Double, Double>(hood040[0], hood040[1]),
            new Pair<Double, Double>(hood050[0], hood050[1]),
            new Pair<Double, Double>(hood060[0], hood060[1]),
            new Pair<Double, Double>(hood070[0], hood070[1]),
            new Pair<Double, Double>(hood080[0], hood080[1]),
            new Pair<Double, Double>(hood090[0], hood090[1]),
            new Pair<Double, Double>(hood100[0], hood100[1]),
            new Pair<Double, Double>(hood110[0], hood110[1]),
    };

    public static Interpolator interpolatorHood = new Interpolator(interpolatorHoodData);

    public static void recalculate_interpolators() {
        interpolatorShooterData = new Pair[]{
                new Pair<Double, Double>(shooter040[0], shooter040[1]),
                new Pair<Double, Double>(shooter050[0], shooter050[1]),
                new Pair<Double, Double>(shooter060[0], shooter060[1]),
                new Pair<Double, Double>(shooter070[0], shooter070[1]),
                new Pair<Double, Double>(shooter080[0], shooter080[1]),
                new Pair<Double, Double>(shooter090[0], shooter090[1]),
                new Pair<Double, Double>(shooter100[0], shooter100[1]),
                new Pair<Double, Double>(shooter110[0], shooter110[1]),
        };

        interpolatorShooter = new Interpolator(interpolatorShooterData);

        interpolatorHoodData = new Pair[]{
                new Pair<Double, Double>(hood040[0], hood040[1]),
                new Pair<Double, Double>(hood050[0], hood050[1]),
                new Pair<Double, Double>(hood060[0], hood060[1]),
                new Pair<Double, Double>(hood070[0], hood070[1]),
                new Pair<Double, Double>(hood080[0], hood080[1]),
                new Pair<Double, Double>(hood090[0], hood090[1]),
                new Pair<Double, Double>(hood100[0], hood100[1]),
                new Pair<Double, Double>(hood110[0], hood110[1]),
        };

        interpolatorHood = new Interpolator(interpolatorHoodData);
    }

    public static double SHOOTER_FF_CLOSE = 1.0;
    public static double SHOOTER_FF_MIDDLE = 0.41;
    public static double SHOOTER_FF_FAR = 0.54;
    public static double SHOOTER_FF_AUTO = 0.5;//0.5 red || 0.56 blue

    public static double SHOOTER_CLOSE = 2.0;
    public static double SHOOTER_MIDDLE = 2.4;
    public static double SHOOTER_FAR = 2.7;
    public static double SHOOTER_AUTO = 2.6; //0.26 for red 2.7
    public static double SHOOTER_RPM_TOLERANCE = 0.15;

    // TURRET
    public static double TURRET_LEFT = 500;
    public static double TURRET_RIGHT = -500;
    public static double TURRET_STATIC = 0.12; //increase when its a little off
    public static double TURRET_KP = 0.006;

    // BRAKE
    public static double BRAKE_DOWN = 0.3;
    public static double BRAKE_UP = 0.1;

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

        mecanumController.setStaticPower(PestoFTCConfig.STATIC_DRIVE);

        if (initializePinpoint) {
//            DeterministicTracker tracker = new NotEnoughOdometryTracker.TrackerBuilder(
//                    hardwareMap,
//                    ODOMETRY_TICKS_PER_INCH,
//                    // X Parameters
//                    new SimpleMatrix(new double[][]{
//                            new double[]{0, 0, 0}
//                    }),
//                    // Y Parameters
//                    new SimpleMatrix(new double[][]{
//                            new double[]{0, 0, 0}
//                    }),
//                    // R Parameters
//                    new SimpleMatrix(new double[][]{
//                            new double[]{-0.234178, -0.000776, 0.231725}
//                    }),
//                    new String[]{
//                            leftName,
//                            centerName,
//                            rightName
//                    },
//                    new DcMotorSimple.Direction[]{
//                            DcMotorSimple.Direction.FORWARD,
//                            DcMotorSimple.Direction.FORWARD,
//                            DcMotorSimple.Direction.FORWARD
//                    }
//            )
//                    .build();

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

//            DeterministicTracker tracker = new ThreeWheelOdometryTracker.TrackerBuilder(
//                    hardwareMap,
//                    ODOMETRY_TICKS_PER_INCH,
//                    ODOMETRY_TICKS_PER_INCH,
//                    ODOMETRY_TICKS_PER_INCH,
//                    FORWARD_OFFSET,
//                    ODOMETRY_WIDTH,
//                    leftName,
//                    centerName,
//                    rightName,
//                    leftDirection,
//                    centerDirection,
//                    rightDirection
//            )
//                    .build();

            TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);
            teleOpController.useTrackerIMU(tracker);

            teleOpController.setSpeedController(gamepad -> gamepad.left_bumper ? 0.6 : 1.0);

//            teleOpController.counteractCentripetalForce(tracker, MAX_FORCE);

            FrontalLobe.teleOpController = teleOpController;
            FrontalLobe.tracker = tracker;
        }

        FrontalLobe.driveController = mecanumController;
    }
}
