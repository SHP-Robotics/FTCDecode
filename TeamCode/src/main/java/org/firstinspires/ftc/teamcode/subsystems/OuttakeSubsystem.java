package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Uses FuzzyPidController (separate file) to regulate shooter velocity.
 * Also smooths derivative implicitly via the controller's internal derivative LPF,
 * and (optionally) smooths measured velocity to reduce discrete jitter.
 */
public class OuttakeSubsystem {
    public CortexLinkedMotor leftShooter;
    public CortexLinkedMotor rightShooter;

    // ----- Controller -----
    public final FuzzyPidController shooterPid;
    private double setPower = 0.0;

    // Optional: smooth measurement to reduce quantization/jitter (discrete encoders)
    private double velFilt = 0.0;

    // Timing
    private long lastNs = 0L;

    // If you still want these exposed for telemetry
    private double error = 0.0;
    private double deSmoothed = 0.0; // from controller's filtered derivative

    public OuttakeSubsystem() {
        leftShooter = MotorCortex.getMotor("leftShooter");
        leftShooter.setMode(RUN_WITHOUT_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShooter = MotorCortex.getMotor("rightShooter");
        rightShooter.setMode(RUN_WITHOUT_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        FuzzyPidController.Config cfg = new FuzzyPidController.Config();
        cfg.kp = 0.01;
        cfg.ki = 0;
        cfg.kd = 0;

        // clamp motor power
        cfg.outMin = -1.0;
        cfg.outMax =  1.0;

        // smooth discrete derivative
        cfg.dTau = 0.03;          // 100ms LPF

        // clamp delta time
        cfg.dtMin = 1e-4;
        cfg.dtMax = 0.1;

        // fuzzy normalization
        cfg.eScale = 800; // average e, e / eScale = eN
        cfg.deScale = 20000; // average de, de / deScale = deN

        // Anti-windup
        cfg.awGain = 0.7;

        // For velocity control, derivative-on-measurement is usually best
        cfg.derivativeOnMeasurement = true;

        shooterPid = new FuzzyPidController(cfg);
    }

    public void reinitialize() {
        leftShooter = MotorCortex.getMotor(0, 2);
        leftShooter.setMode(RUN_USING_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        rightShooter = MotorCortex.getMotor(1, 2);
        rightShooter.setMode(RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterPid.reset();
        lastNs = 0L;
        velFilt = 0.0;
        error = 0.0;
        deSmoothed = 0.0;
    }

    public void setPowerDirect(double power) {
        leftShooter.setPowerResult(power);
        rightShooter.setPowerResult(power);
    }

    public void setPower(double power) {
        this.setPower = power;
    }

    public double getRPM() {
        return leftShooter.getVelocity(AngleUnit.RADIANS);
    }

    public double getVelocity() {
        return (Math.abs(leftShooter.getVelocity()) + Math.abs(rightShooter.getVelocity())) / 2;
    }

    public double getExpectedVelocity(double power) {
        return leftShooter.getMotorType().getAchieveableMaxTicksPerSecondRounded() * Math.abs(power);
    }

    public boolean isBusy(double power) {
        double expectedTPS = getExpectedVelocity(power);
        double TPS = getVelocity();
        return (expectedTPS - TPS) / expectedTPS > 0.01;
    }

    /**
     * Closed-loop update:
     * - Computes target velocity from current commanded power (your existing behavior).
     * - Smooths measurement (optional) to reduce discrete encoder jitter.
     * - Runs fuzzy PID, which already smooths derivative internally (dTau).
     * - Applies correction around the existing feedforward power.
     */
    public void update() {
        if (setPower == 0) {
            this.setPowerDirect(0);
            return;
        }

        long now = System.nanoTime();
        if (lastNs == 0L) {
            lastNs = now;
            velFilt = getVelocity();
            return;
        }

        double dt = (now - lastNs) * 1e-9;
        lastNs = now;

        // Target from current commanded power (keep your intent: PID trims around it)
        double basePower = Math.abs(setPower);
        double targetVel = getExpectedVelocity(basePower);

        // --- Measurement smoothing (discrete space jitter reducer) ---
        // 1st-order low-pass on velocity
        // Pick tauVel small enough to not lag badly; 10-30ms is typical.
        final double tauVel = 0.02; // 20ms
        double alpha = dt / (tauVel + dt);
        double vel = getVelocity();
        velFilt += alpha * (vel - velFilt);

        // Error
        error = targetVel - velFilt;

        // --- Fuzzy PID correction ---
        // Using updateFromError so we can pass measurement for derivative-on-measurement mode.
        // The controller’s internal derivative is already smoothed (dTau), so de is not spiky.
        double correction = shooterPid.updateFromError(error, velFilt, dt);

        // Apply as trim around base power (clamped inside PID, but clamp final anyway)
        double outPower = clamp(basePower + correction, -1.0, 1.0);

        leftShooter.setPowerResult(outPower * Math.signum(setPower));
        rightShooter.setPowerResult(outPower * Math.signum(setPower));

        // For telemetry: controller’s smoothed derivative signal (already filtered)
        deSmoothed = shooterPid.getFilteredDerivative();
    }

    // Optional telemetry getters
    public double getError() { return error; }
    public double getDeSmoothed() { return deSmoothed; }
    public double getKpEff() { return shooterPid.getKpEffective(); }
    public double getKiEff() { return shooterPid.getKiEffective(); }
    public double getKdEff() { return shooterPid.getKdEffective(); }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}