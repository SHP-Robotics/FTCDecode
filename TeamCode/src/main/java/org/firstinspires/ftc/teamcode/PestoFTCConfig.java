package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.trackers.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.Cerebrum;
import com.shprobotics.pestocore.processing.ConfigInterface;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoConfig;

@Config
@PestoConfig()
public class PestoFTCConfig implements ConfigInterface {
    public static boolean initialized = false; // don't mess with this :O
    public static boolean initializePinpoint = false;

    // ODOMETRY
    public static String leftName = "fl";
    public static String centerName = "bl";
    public static String rightName = "fr";

    public static DcMotorSimple.Direction leftDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction centerDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightDirection = DcMotorSimple.Direction.REVERSE;

    public static double ODOMETRY_TICKS_PER_INCH = 505.3169;
    public static double FORWARD_OFFSET = -10;
    public static double ODOMETRY_WIDTH = 9.663;

    // DROPDOWN
    public static double DROPDOWN_DRIVE = 0.577;
    public static double DROPDOWN_INTAKE = 0.668;
    public static double DROPDOWN_PUSH = 0.420;

    // INDEXER
    public static double INDEXER_OUTTAKE = 0.08;
    public static double INDEXER_BLOCK = 0.26;

    // HOOD
    public static double HOOD_CLOSE = 0.00;
    public static double HOOD_MID = 0.0551;
    public static double HOOD_FAR = 0.1131;

    public static void initialize(HardwareMap hardwareMap) {
        MotorCortex.initialize(hardwareMap);
        Cerebrum.initialize();

        CortexLinkedMotor frontLeft = MotorCortex.getMotor("fl");
        CortexLinkedMotor frontRight = MotorCortex.getMotor("fr");
        CortexLinkedMotor backLeft = MotorCortex.getMotor("bl");
        CortexLinkedMotor backRight = MotorCortex.getMotor("br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        DriveController driveController = new MecanumController(
                MotorCortex.getMotor("fl"),
                MotorCortex.getMotor("fr"),
                MotorCortex.getMotor("bl"),
                MotorCortex.getMotor("br")
        );

        driveController.configureMotorDirections(new DcMotorSimple.Direction[]{
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        });

        driveController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (initializePinpoint) {
            DeterministicTracker tracker = new ThreeWheelOdometryTracker.TrackerBuilder(
                    hardwareMap,
                    ODOMETRY_TICKS_PER_INCH,
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

            teleOpController.setSpeedController(gamepad -> 1.0);

//            teleOpController.counteractCentripetalForce();

            FrontalLobe.teleOpController = teleOpController;
            FrontalLobe.tracker = tracker;
        }

        FrontalLobe.driveController = driveController;
    }
}
