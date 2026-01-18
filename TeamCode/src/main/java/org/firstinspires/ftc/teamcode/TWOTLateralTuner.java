package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.PestoDashCore;
import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.QualitativeSelector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.drivebases.trackers.TWOT;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoTelemetry;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

@TeleOp(name = "TWOT Lateral Tuner")
public class TWOTLateralTuner extends LinearOpMode {
    double distance = 24.5;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);
        PestoTelemetry pestoTelemetry = FrontalLobe.pestoTelemetry;

        QualitativeSelector recordingSelector = new QualitativeSelector("recording?", "no", new String[]{"no", "yes"});
        pestoTelemetry.addToDash(recordingSelector);
        pestoTelemetry.update();

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
                        new double[]{0.0},
                        new double[]{0.0},
                        new double[]{0.0},
                        new double[]{0.0},
                        new double[]{0.0},
                        new double[]{0.0},
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

        telemetry.addLine("Prereq. Run TWOT Rotation Tuner");
        telemetry.addLine("1. Press a to start recording");
        telemetry.addLine("2. Push the robot " + distance + " inches right (no vertical movement)");
        telemetry.addLine("3. Press b to stop recording");
        telemetry.addLine("4. Repeat to get Matrix Values m11, m21, m31, m41, m51, m61");
        telemetry.update();

        ArrayList<double[]> T_arr = new ArrayList<>();

        double m11 = 0.0;
        double m21 = 0.0;
        double m31 = 0.0;
        double m41 = 0.0;
        double m51 = 0.0;
        double m61 = 0.0;

        waitForStart();

        tracker.reset();
        while (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive() && !isStopRequested() && !gamepad1.a) {
                pestoTelemetry.update();

                DataItem item = PestoDashCore.getItem(recordingSelector.getId());

                if (item != null && item.getValue() != null) {
                    String selection = (String) item.getValue();
                    if (selection.equals("yes"))
                        break;
                }

                telemetry.addLine("Prereq. Run TWOT Rotation Tuner");
                telemetry.addLine("1. Press a to start recording");
                telemetry.addLine("2. Push the robot " + distance + " inches right (no vertical movement)");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get Matrix Values m11, m21, m31, m41, m51, m61");
                telemetry.addLine();
                telemetry.addLine("Waiting for A");
                telemetry.addLine();
                telemetry.addData("m11", m11);
                telemetry.addData("m21", m21);
                telemetry.addData("m31", m31);
                telemetry.addData("m41", m41);
                telemetry.addData("m51", m51);
                telemetry.addData("m61", m61);
                telemetry.update();

                pestoTelemetry.clear();
                pestoTelemetry.addToDash("m11", m11);
                pestoTelemetry.addToDash("m21", m21);
                pestoTelemetry.addToDash("m31", m31);
                pestoTelemetry.addToDash("m41", m41);
                pestoTelemetry.addToDash("m51", m51);
                pestoTelemetry.addToDash("m61", m61);
            }

            double TdLCos = 0.0;
            double TdLSin = 0.0;
            double TdCCos = 0.0;
            double TdCSin = 0.0;
            double TdRCos = 0.0;
            double TdRSin = 0.0;

            double field_x = 0.0;
            double field_y = 0.0;
            double heading = 0.0;

            MotorCortex.update();
            tracker.reset();
            tracker.leftOdometry.getInchesTravelled();
            tracker.centerOdometry.getInchesTravelled();
            tracker.rightOdometry.getInchesTravelled();

            while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
                pestoTelemetry.update();

                DataItem item = PestoDashCore.getItem(recordingSelector.getId());

                if (item != null && item.getValue() != null) {
                    String selection = (String) item.getValue();
                    if (selection.equals("no"))
                        break;
                }

                MotorCortex.update();

                double dL = tracker.leftOdometry.getInchesTravelled();
                double dC = tracker.centerOdometry.getInchesTravelled();
                double dR = tracker.rightOdometry.getInchesTravelled();

                SimpleMatrix r_inputs = new SimpleMatrix(new double[][]{
                        new double[]{dL, dC, dR}
                });

                double r = r_inputs.mult(tracker.ODOMETRY_PARAMETERS_R).toArray2()[0][0];

                double cos = Math.cos(heading);
                double sin = Math.sin(heading);

                SimpleMatrix xy_inputs = new SimpleMatrix(new double[][]{
                        new double[]{
                                dL * cos,
                                dL * sin,
                                dC * cos,
                                dC * sin,
                                dR * cos,
                                dR * sin
                        }
                });

                double x = xy_inputs.mult(tracker.ODOMETRY_PARAMETERS_X).toArray2()[0][0];
                double y = xy_inputs.mult(tracker.ODOMETRY_PARAMETERS_Y).toArray2()[0][0];

                field_x += x;
                field_y += y;
                heading += r;

                TdLCos += (dL * cos);
                TdLSin += (dL * sin);
                TdCCos += (dC * cos);
                TdCSin += (dC * sin);
                TdRCos += (dR * cos);
                TdRSin += (dR * sin);

                telemetry.addLine("1. Recording!");
                telemetry.addLine();
                telemetry.addLine("2. Push the robot " + distance + " inches right (no vertical movement)");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get Matrix Values m11, m21, m31, m41, m51, m61");

                telemetry.update();

                pestoTelemetry.clear();
                pestoTelemetry.addToDash("X", field_x);
                pestoTelemetry.addToDash("Y", field_y);
                pestoTelemetry.addToDash("R", heading);
            }

            T_arr.add(new double[]{TdLCos, TdLSin, TdCCos, TdCSin, TdRCos, TdRSin});
            SimpleMatrix T_matrix = new SimpleMatrix(T_arr.size(), 6);

            SimpleMatrix translations = new SimpleMatrix(T_arr.size(), 1);
            translations.fill(distance);

            for (int i = 0; i < T_arr.size(); i++) {
                T_matrix.setRow(i, new SimpleMatrix(T_arr.get(i)));
            }
            
            SimpleMatrix translational_parameters = T_matrix.pseudoInverse().mult(translations);
            
            m11 = translational_parameters.get(0, 0);
            m21 = translational_parameters.get(1, 0);
            m31 = translational_parameters.get(2, 0);
            m41 = translational_parameters.get(3, 0);
            m51 = translational_parameters.get(4, 0);
            m61 = translational_parameters.get(5, 0);

            tracker.ODOMETRY_PARAMETERS_X.set(0, 0, m11);
            tracker.ODOMETRY_PARAMETERS_X.set(1, 0, m21);
            tracker.ODOMETRY_PARAMETERS_X.set(2, 0, m31);
            tracker.ODOMETRY_PARAMETERS_X.set(3, 0, m41);
            tracker.ODOMETRY_PARAMETERS_X.set(4, 0, m51);
            tracker.ODOMETRY_PARAMETERS_X.set(5, 0, m61);

        }
    }
};