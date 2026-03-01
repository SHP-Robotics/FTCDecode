package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.FuzzyPidController;

//@Disabled
@Config
@TeleOp(name = "Fuzzy PID Tuner")
public class PIDFTuner extends BaseRobot {
    public static double shooter_power = 0.0;

    public static double kp = 0.01;
    public static double ki = 0.0;
    public static double kd = 0.0;

    public static double outMin = -1.0;
    public static double outMax = 1.0;

    public static double dTau = 0.03;

    public static double dtMin = 1e-4;
    public static double dtMax = 0.1;

    public static double eScale = 800;
    public static double deScale = 20000;

    public static double awGain = 0.7;

    public static boolean derivativeOnMeasurement = true;

    @Override
    public void runOpMode() {
        PestoFTCConfig.initializePinpoint = true;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        super.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FrontalLobe.update();
            MotorCortex.update();

            FuzzyPidController.Config cfg = new FuzzyPidController.Config();
            cfg.kp = kp;
            cfg.ki = ki;
            cfg.kd = kd;

            // clamp motor power
            cfg.outMin = outMin;
            cfg.outMax = outMax;

            // smooth discrete derivative
            cfg.dTau = dTau;          // 100ms LPF

            // clamp delta time
            cfg.dtMin = dtMin;
            cfg.dtMax = dtMax;

            // fuzzy normalization
            cfg.eScale = eScale; // average e, e / eScale = eN
            cfg.deScale = deScale; // average de, de / deScale = deN

            // Anti-windup
            cfg.awGain = awGain;

            // For velocity control, derivative-on-measurement is usually best
            cfg.derivativeOnMeasurement = derivativeOnMeasurement;

            outtakeSubsystem.shooterPid.setCfg(cfg);

            outtakeSubsystem.setPower(shooter_power);
            outtakeSubsystem.update();
            dashboardTelemetry.addData("expected velocity", outtakeSubsystem.getExpectedVelocity(shooter_power));
            dashboardTelemetry.addData("velocity", outtakeSubsystem.getVelocity());
            dashboardTelemetry.update();
        }
    }
}
