package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.algorithms.Constants;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.trackers.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.processing.Cerebrum;
import com.shprobotics.pestocore.processing.ConfigInterface;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoConfig;

@Config
@PestoConfig()
public class PestoFTCConfig implements ConfigInterface {
    private static boolean initialized = false; // don't mess with this :O
    public static boolean initializePinpoint = true;

    public static double mass = 21.7; // lbs
    public static double MAX_FORCE = 2220;
    public static double DECELERATION = 35;

    // ODOMETRY
    private static String leftName = "backLeft";
    private static String centerName = "frontLeft";
    private static String rightName = "backRight";

    private static DcMotorSimple.Direction leftDirection = DcMotorSimple.Direction.REVERSE;
    private static DcMotorSimple.Direction centerDirection = DcMotorSimple.Direction.REVERSE;
    private static DcMotorSimple.Direction rightDirection = DcMotorSimple.Direction.REVERSE;

    private static double ODOMETRY_TICKS_PER_INCH_CENTER = 505.3169;
    private static double ODOMETRY_TICKS_PER_INCH_LEFT = 505.3169;
    private static double ODOMETRY_TICKS_PER_INCH_RIGHT = 505.3169;
    public static double FORWARD_OFFSET = -1.565;
    public static double ODOMETRY_WIDTH = 9.1474;


    // DROPDOWN
    public static double DROPDOWN_DRIVE = 35; // 0.29;
    public static double DROPDOWN_INTAKE = 85; // 0.47;
    public static double DROPDOWN_PUSH = 35; // 0.29;
    public static double DROPDOWN_PUSH_AUTO = 35; // 0.29;

    // INDEXER
    public static double INDEXER_OUTTAKE = 0.10;
    public static double INDEXER_BLOCK = 0.24;

    // HOOD
    public static double HOOD_CLOSE = 0.01;
    public static double HOOD_MID = 0.04;
    public static double HOOD_FAR = 0.01;
    public static double HOOD_AUTO_FAR = 0.04;

    // SHOOTER
    public static double SHOOTER_CLOSE = 0.70;
    public static double SHOOTER_MIDDLE = 0.87;
    public static double SHOOTER_FAR = 1.0;
    public static double AUTO_FAR = 1.0;

    // CAMERA
    public static double STATIC_DRIVE = 0.1;
    public static double KP = 0.015;

    // AUTO
    public static double HEADING_KP = 8;
    public static double ENDPOINT_KP = 0.5;

    public static void initialize(HardwareMap hardwareMap) {
        MotorCortex.initialize(hardwareMap);
        Cerebrum.initialize();

        MotorCortex.getMotor("3");

        MecanumController driveController = new MecanumController(
                MotorCortex.getMotor("frontLeft"),
                MotorCortex.getMotor("frontRight"),
                MotorCortex.getMotor("backLeft"),
                MotorCortex.getMotor("backRight")
        );

        driveController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        });

//        driveController.setPowerVectors(new Vector2D[]{
//                new Vector2D(Math.min(FORWARD_VELOCITY / STRAFE_VELOCITY, 1.0), Math.min(STRAFE_VELOCITY / FORWARD_VELOCITY, 1.0)),
//                new Vector2D(-Math.min(FORWARD_VELOCITY / STRAFE_VELOCITY, 1.0), Math.min(STRAFE_VELOCITY / FORWARD_VELOCITY, 1.0)),
//                new Vector2D(-Math.min(FORWARD_VELOCITY / STRAFE_VELOCITY, 1.0), Math.min(STRAFE_VELOCITY / FORWARD_VELOCITY, 1.0)),
//                new Vector2D(Math.min(FORWARD_VELOCITY / STRAFE_VELOCITY, 1.0), Math.min(STRAFE_VELOCITY / FORWARD_VELOCITY, 1.0))
//        });

        driveController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        driveController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        driveController.frontLeft.setAccelerationMax(5.0);
//        driveController.frontRight.setAccelerationMax(5.0);
//        driveController.backLeft.setAccelerationMax(5.0);
//        driveController.backRight.setAccelerationMax(5.0);

        driveController.frontLeft.setAccelerationMax(Double.POSITIVE_INFINITY);
        driveController.frontRight.setAccelerationMax(Double.POSITIVE_INFINITY);
        driveController.backLeft.setAccelerationMax(Double.POSITIVE_INFINITY);
        driveController.backRight.setAccelerationMax(Double.POSITIVE_INFINITY);

        if (initializePinpoint) {
            DeterministicTracker tracker = new ThreeWheelOdometryTracker.TrackerBuilder(
                    hardwareMap,
                    ODOMETRY_TICKS_PER_INCH_LEFT,
                    ODOMETRY_TICKS_PER_INCH_RIGHT,
                    ODOMETRY_TICKS_PER_INCH_CENTER,
                    FORWARD_OFFSET,
                    ODOMETRY_WIDTH,
                    leftName,
                    centerName,
                    rightName,
                    leftDirection,
                    centerDirection,
                    rightDirection
            )
                    .build();

            TeleOpController teleOpController = new TeleOpController(driveController, hardwareMap);
            teleOpController.useTrackerIMU(tracker);

            teleOpController.setSpeedController(gamepad -> gamepad.left_bumper ? 0.6 : 1.0);

            teleOpController.counteractCentripetalForce(tracker, MAX_FORCE);

            FrontalLobe.teleOpController = teleOpController;
            FrontalLobe.tracker = tracker;
        }

        FrontalLobe.driveController = driveController;
        Constants.mass = mass;
    }
}
