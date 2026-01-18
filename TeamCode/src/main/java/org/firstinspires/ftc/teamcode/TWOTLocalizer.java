package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.drivebases.trackers.TWOT;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoTelemetry;

import org.ejml.simple.SimpleMatrix;

@TeleOp(name = "TWOT Localizer")
public class TWOTLocalizer extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);
        PestoTelemetry pestoTelemetry = FrontalLobe.pestoTelemetry;

        TWOT tracker = new TWOT.TrackerBuilder(
                hardwareMap,
                505.3169,
                new SimpleMatrix(new double[][]{
                        new double[]{0.0},
                        new double[]{0.0},
                        new double[]{0.0},
                        new double[]{0.0},
                        new double[]{0.0},
                        new double[]{0.0},
                }),
                new SimpleMatrix(new double[][]{
                        new double[]{-3.403336515717541},
                        new double[]{1.2699958957478223},
                        new double[]{1.3460775162359155},
                        new double[]{-1.0179996295954816},
                        new double[]{4.3829921388933},
                        new double[]{3.4734383975380965},
                }),
                new SimpleMatrix(new double[][]{
                        new double[]{-0.11201892953178422},
                        new double[]{-0.0034454197797296054},
                        new double[]{0.11231265526595093},
                }),
                "backLeft",
                "frontLeft",
                "backRight",
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE
        )
                .build();

        waitForStart();

        tracker.reset();
        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();

            telemetry.addData("X", tracker.getCurrentPosition().getX());
            telemetry.addData("Y", tracker.getCurrentPosition().getY());
            telemetry.addData("R", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.update();

            pestoTelemetry.clear();
            pestoTelemetry.addToDash("X", tracker.getCurrentPosition().getX());
            pestoTelemetry.addToDash("Y", tracker.getCurrentPosition().getY());
            pestoTelemetry.addToDash("R", tracker.getCurrentPosition().getHeadingRadians());
            pestoTelemetry.update();
        }
    }
};