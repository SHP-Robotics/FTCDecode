package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.algorithms.Constants;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.processing.Cerebrum;
import com.shprobotics.pestocore.processing.ConfigInterface;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoConfig;

@Config
@PestoConfig()
public class PestoFTCConfig implements ConfigInterface {
    public static boolean initialized = false; // don't mess with this :O
    public static boolean initializePinpoint = true;

    public static double mass = 23.7; // lbs
    public static double MAX_FORCE = 2170;
    public static double DECELERATION = 100;
    public static double ENDPOINT_KP = 0.3;
    public static double HEADING_KP = 4;

    // ODOMETRY
    public static String leftName = "backL";
    public static String centerName = "backR";
    public static String rightName = "frontR";

    public static DcMotorSimple.Direction leftDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction centerDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightDirection = DcMotorSimple.Direction.FORWARD;

    public static double ODOMETRY_TICKS_PER_INCH = 505.3169;
    public static double FORWARD_OFFSET = -1.9;
    public static double ODOMETRY_WIDTH = 4.1789;

    public static double STATIC_DRIVE = 0.22;

    // DROPDOWN
    public static double DROPDOWN_DRIVE = 0.4;
    public static double DROPDOWN_INTAKE = 0.25;
    public static double DROPDOWN_PUSH = 0.35;

    // BLOCKER
    public static double BLOCKER_BLOCK = 0.4;
    public static double BLOCKER_OUTTAKE = 0.7;

    // INDEXER
    public static double INDEXER_IN = 0.20;
    public static double INDEXER_OUTISH = 0.52;
    public static double INDEXER_OUT = 0.589;

    // HOOD
    public static double HOOD_CLOSE = 0.33;
    public static double HOOD_MID = 0.34;
    public static double HOOD_FAR = 0.35;

    // SHOOTER
    public static double SHOOTER_KP = 0.3;

    public static double SHOOTER_FF_CLOSE = 0.32;
    public static double SHOOTER_FF_MIDDLE = 0.5;
    public static double SHOOTER_FF_FAR = 0.5;
    public static double SHOOTER_FF_AUTO = 0.5;

    public static double SHOOTER_CLOSE = 1.8;
    public static double SHOOTER_MIDDLE = 3.5;
    public static double SHOOTER_FAR = 2.6;
    public static double SHOOTER_AUTO = 2.6;

    public static double SHOOTER_RPM_TOLERANCE = 0.15;

    // TURRET
    public static double TURRET_LEFT = 100;
    public static double TURRET_RIGHT = -100;
    public static double TURRET_STATIC = 0.12; //increase when its a little off
    public static double TURRET_KP = 0.006;

    // BRAKE
    public static double BRAKE_DOWN = 0.3;
    public static double BRAKE_UP = 0.1;

    public static void initialize(HardwareMap hardwareMap) {
        MotorCortex.initialize(hardwareMap);
        Cerebrum.initialize();

        MecanumController mecanumController = new MecanumController(
                MotorCortex.getMotor("frontLeft"),
                MotorCortex.getMotor("frontRight"),
                MotorCortex.getMotor("backLeft"),
                MotorCortex.getMotor("backRight")
        );

        FrontalLobe.driveController = mecanumController;

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        });

        mecanumController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ThreeWheelOdometryTracker tracker = new ThreeWheelOdometryTracker.TrackerBuilder(
                hardwareMap,
                ODOMETRY_TICKS_PER_INCH,
                ODOMETRY_TICKS_PER_INCH,
                ODOMETRY_TICKS_PER_INCH,
                -2.0,
                4.9299,
                "backRight",
                "frontRight",
                "backLeft",
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD
            )
                .build();

        FrontalLobe.tracker = tracker;

        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);
        FrontalLobe.teleOpController = teleOpController;
        teleOpController.useTrackerIMU(tracker);

        // TODO: set speed controller lambda
        // b is circle
        // a is cross
        // x is square
        // y is triangle
        // left_bumper
        // right_bumper
        // left_trigger > 0.05
        // right_trigger > 0.05
        // dpad_down
        // dpad_left
        // dpad_right
        // dpad_up
        // touchpad
        teleOpController.setSpeedController(gamepad -> gamepad.x ? 0.6 : 1.0);

        FrontalLobe.driveController = mecanumController;
        Constants.mass = mass;
    }
}
