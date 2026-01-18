package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.PestoDashCore;
import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.QualitativeSelector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.drivebases.trackers.Odometry;
import com.shprobotics.pestocore.drivebases.trackers.TWOT;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;
import com.shprobotics.pestocore.processing.PestoTelemetry;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

@TeleOp(name = "TWOT Rotation Tuner")
public class TWOTRotationTuner extends LinearOpMode {
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
        ).build();

        Odometry left = tracker.leftOdometry;
        Odometry right = tracker.rightOdometry;
        Odometry center = tracker.centerOdometry;

        telemetry.addLine("1. Press a to start recording");
        telemetry.addLine("2. Rotate the robot 180 degrees");
        telemetry.addLine("3. Press b to stop recording");
        telemetry.addLine("4. Repeat to get Matrix Values m13, m23, m33");
        telemetry.update();

        FrontalLobe.pestoTelemetry.addToDash(new QualitativeSelector("recording?", "no", new String[]{"no", "yes"}));
        FrontalLobe.pestoTelemetry.update();

        ArrayList<double[]> T_arr = new ArrayList<>();

        double m13 = 0.0;
        double m23 = 0.0;
        double m33 = 0.0;

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
                telemetry.addLine("2. Rotate the robot 180 degrees");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get Matrix Values m13, m23, m33");
                telemetry.addLine();
                telemetry.addLine("Waiting for A");
                telemetry.addLine();
                telemetry.addData("m13", m13);
                telemetry.addData("m23", m23);
                telemetry.addData("m33", m33);
                telemetry.update();

                pestoTelemetry.clear();
                pestoTelemetry.addToDash("m13", m13);
                pestoTelemetry.addToDash("m23", m23);
                pestoTelemetry.addToDash("m33", m33);
            }

            double TdL = 0.0;
            double TdC = 0.0;
            double TdR = 0.0;

            double R = 0.0;

            while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
                pestoTelemetry.update();

                DataItem item = PestoDashCore.getItem(recordingSelector.getId());

                if (item != null && item.getValue() != null) {
                    String selection = (String) item.getValue();
                    if (selection.equals("no"))
                        break;
                }

                MotorCortex.update();

                double dL = left.getInchesTravelled();
                double dC = center.getInchesTravelled();
                double dR = right.getInchesTravelled();

                TdL += dL;
                TdC += dC;
                TdR += dR;

                R += (dL * m13) + (dC * m23) + (dR * m33);

                telemetry.addLine("1. Recording!");
                telemetry.addLine();
                telemetry.addLine("2. Rotate the robot 180 degrees");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get Matrix Values m13, m23, m33");
                telemetry.update();

                pestoTelemetry.clear();
                pestoTelemetry.addToDash("r", R);
            }

            T_arr.add(new double[]{TdL, TdC, TdR});
            SimpleMatrix T_matrix = new SimpleMatrix(T_arr.size(), 3);

            SimpleMatrix rotations = new SimpleMatrix(T_arr.size(), 1);
            rotations.fill(Math.PI);

            for (int i = 0; i < T_arr.size(); i++) {
                T_matrix.setRow(i, new SimpleMatrix(T_arr.get(i)));
            }

            SimpleMatrix rotation_parameters = T_matrix.pseudoInverse().mult(rotations);

            m13 = rotation_parameters.get(0, 0);
            m23 = rotation_parameters.get(1, 0);
            m33 = rotation_parameters.get(2, 0);
        }
    }
};