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

@TeleOp(name = "TWOT Rotation Tuner", group = "Pesto EJML Tuners")
public class TWOTRotationTuner extends LinearOpMode {
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

        telemetry.addLine("1. Press a to start recording");
        telemetry.addLine("1. Rotate the robot 360 degrees");
        telemetry.addLine("2. Press b to stop recording");
        telemetry.addLine("3. Repeat to get Matrix Values m13, m23, m33");
        telemetry.update();

        ArrayList<double[]> T_arr = new ArrayList<>();

        double m13 = 0.0;
        double m23 = 0.0;
        double m33 = 0.0;

        waitForStart();

        tracker.reset();
        while (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive() && !isStopRequested() && !gamepad1.a) {
                telemetry.addLine("Prereq. Run TWOT Rotation Tuner");
                telemetry.addLine("1. Press a to start recording");
                telemetry.addLine("2. Rotate the robot 360 degrees");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get Matrix Values m13, m23, m33");
                telemetry.addLine();
                telemetry.addLine("Waiting for A");
                telemetry.addLine();
                telemetry.addData("m13", m13);
                telemetry.addData("m23", m23);
                telemetry.addData("m33", m33);
                telemetry.update();
            }

            double TdL = 0.0;
            double TdC = 0.0;
            double TdR = 0.0;

            while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
                MotorCortex.update();

                TdL += left.getInchesTravelled();
                TdC += right.getInchesTravelled();
                TdR += center.getInchesTravelled();

                telemetry.addLine("1. Recording!");
                telemetry.addLine();
                telemetry.addLine("2. Rotate the robot 360 degrees");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get Matrix Values m13, m23, m33");
                telemetry.update();
            }

            T_arr.add(new double[]{TdL, TdC, TdR});
            SimpleMatrix T_matrix = new SimpleMatrix(T_arr.size(), 3);

            SimpleMatrix rotations = new SimpleMatrix(T_arr.size(), 1);
            rotations.fill(Math.PI * 2);

            for (int i = 0; i < T_arr.size(); i++) {
                T_matrix.setRow(i, new SimpleMatrix(T_arr.get(i)));
            }
            
            SimpleMatrix rotation_parameters = rotations.mult(T_matrix.pseudoInverse());
            
            m13 = rotation_parameters.get(0, 2);
            m23 = rotation_parameters.get(1, 2);
            m33 = rotation_parameters.get(2, 2);
        }
    }
};