package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.drivebases.trackers.Odometry;
import com.shprobotics.pestocore.drivebases.trackers.TWOT;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

@TeleOp(name = "TWOT Translational Tuner", group = "Pesto EJML Tuners")
public class TWOTTranslationalTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        TWOT tracker = new TWOT.TrackerBuilder(
                hardwareMap,
                505.3169,
                new SimpleMatrix(new double[][]{
                        new double[]{0.0, 0.0, 0.0},
                        new double[]{0.0, 0.0, 0.0},
                        new double[]{0.0, 0.0, 0.0},
                }),
                "backL",
                "backR",
                "frontR",
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        )
                .build();

        Odometry left = tracker.leftOdometry;
        Odometry right = tracker.rightOdometry;
        Odometry center = tracker.centerOdometry;

        telemetry.addLine("Prereq. Run TWOT Rotation Tuner");
        telemetry.addLine("1. Press a to start recording");
        telemetry.addLine("2. Push the robot 30 inches forward (no horizontal movement)");
        telemetry.addLine("3. Press b to stop recording");
        telemetry.addLine("4. Repeat to get Matrix Values m11, m12, m21, m22, m31, m32");
        telemetry.update();

        ArrayList<double[]> T_arr = new ArrayList<>();

        double m11 = 0.0;
        double m12 = 0.0;
        double m21 = 0.0;
        double m22 = 0.0;
        double m31 = 0.0;
        double m32 = 0.0;

        waitForStart();

        tracker.reset();
        while (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive() && !isStopRequested() && !gamepad1.a) {
                telemetry.addLine("Prereq. Run TWOT Rotation Tuner");
                telemetry.addLine("1. Press a to start recording");
                telemetry.addLine("2. Push the robot 30 inches forward (no horizontal movement)");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get Matrix Values m11, m12, m21, m22, m31, m32");
                telemetry.addLine();
                telemetry.addLine("Waiting for A");
                telemetry.addLine();
                telemetry.addData("m11", m11);
                telemetry.addData("m12", m12);
                telemetry.addData("m21", m21);
                telemetry.addData("m22", m22);
                telemetry.addData("m31", m31);
                telemetry.addData("m32", m32);
                telemetry.update();
            }

            double TdLCos = 0.0;
            double TdLSin = 0.0;
            double TdCCos = 0.0;
            double TdCSin = 0.0;
            double TdRCos = 0.0;
            double TdRSin = 0.0;

            while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
                MotorCortex.update();

                double heading = tracker.getCurrentPosition().getHeadingRadians();
                double cos = Math.cos(heading);
                double sin = Math.sin(heading);

                TdLCos += (left.getInchesTravelled() * cos);
                TdLSin += (left.getInchesTravelled() * sin);
                TdCCos += (center.getInchesTravelled() * cos);
                TdCSin += (center.getInchesTravelled() * sin);
                TdRCos += (right.getInchesTravelled() * cos);
                TdRSin += (right.getInchesTravelled() * sin);

                telemetry.addLine("1. Recording!");
                telemetry.addLine();
                telemetry.addLine("2. Push the robot 30 inches forward (no horizontal movement)");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get Matrix Values m11, m12, m21, m22, m31, m32");

                telemetry.update();
            }

            T_arr.add(new double[]{TdLCos, TdLSin, TdCCos, TdCSin, TdRCos, TdRSin});
            SimpleMatrix T_matrix = new SimpleMatrix(T_arr.size(), 6);

            SimpleMatrix rotations = new SimpleMatrix(T_arr.size(), 1);
            rotations.fill(Math.PI * 2);

            for (int i = 0; i < T_arr.size(); i++) {
                T_matrix.setRow(i, new SimpleMatrix(T_arr.get(i)));
            }
            
            SimpleMatrix rotation_parameters = rotations.mult(T_matrix.pseudoInverse());
            
            m11 = rotation_parameters.get(0, 2);
            m12 = rotation_parameters.get(1, 2);
            m21 = rotation_parameters.get(2, 2);
            m22 = rotation_parameters.get(3, 2);
            m31 = rotation_parameters.get(4, 2);
            m32 = rotation_parameters.get(5, 2);
        }
    }
};