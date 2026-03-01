package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Objects;

public final class FuzzyPidController {
    public static final class Config {
        // Base gains (the fuzzy system scales these each update)
        public double kp = 0.0;
        public double ki = 0.0;
        public double kd = 0.0;

        // Output clamp
        public double outMin = -1.0;
        public double outMax =  1.0;

        // Integral clamp (in "integral state" units, i.e., before multiplying by Ki)
        public double iMin = -Double.POSITIVE_INFINITY;
        public double iMax =  Double.POSITIVE_INFINITY;

        // Derivative low-pass filter time constant (seconds).
        // Smaller -> more responsive, bigger -> more noise rejection.
        public double dTau = 0.02;

        // dt clamp to stay stable with discrete/fast loops
        public double dtMin = 1e-4;  // 0.1 ms
        public double dtMax = 0.1;   // 100 ms

        // Normalization scales for fuzzy inputs
        // (tune these for your system so "medium" error and "medium" error-rate land around 1.0)
        public double eScale = 1.0;
        public double deScale = 1.0;

        // Anti-windup back-calculation strength (0 disables back-calc; typical 0.1..2)
        public double awGain = 0.5;

        // If true, derivative is taken on measurement (more noise-safe for step setpoints).
        // If false, derivative is taken on error.
        public boolean derivativeOnMeasurement = true;

        public Config() {}
    }

    // ===== Internal state =====

    private org.firstinspires.ftc.teamcode.subsystems.FuzzyPidController.Config cfg;

    private double integrator = 0.0;
    private double prevError = 0.0;
    private double prevMeas = 0.0;
    private double dFilt = 0.0;          // filtered derivative signal (either de/dt or -dy/dt)
    private boolean first = true;

    // Optional: expose live gains
    private double kpEff, kiEff, kdEff;

    public FuzzyPidController(org.firstinspires.ftc.teamcode.subsystems.FuzzyPidController.Config config) {
        this.cfg = Objects.requireNonNull(config, "config");
        validateConfig(config);
        this.kpEff = config.kp;
        this.kiEff = config.ki;
        this.kdEff = config.kd;
    }

    public void reset() {
        integrator = 0.0;
        prevError = 0.0;
        prevMeas = 0.0;
        dFilt = 0.0;
        first = true;
    }

    public void setCfg(org.firstinspires.ftc.teamcode.subsystems.FuzzyPidController.Config config) {
        this.cfg = Objects.requireNonNull(config, "config");
        validateConfig(config);
    }

    /**
     * Update using (setpoint, measurement) with explicit dt (seconds).
     */
    public double update(double setpoint, double measurement, double dtSeconds) {
        double e = setpoint - measurement;
        return updateFromError(e, measurement, dtSeconds);
    }

    /**
     * Update using error directly with explicit dt (seconds).
     * If derivativeOnMeasurement is enabled, you must also pass measurement (or something proportional)
     * so the controller can compute d/dt(measurement). If you don't have that, set derivativeOnMeasurement=false.
     */
    public double updateFromError(double error, double measurement, double dtSeconds) {
        double dt = clamp(dtSeconds, cfg.dtMin, cfg.dtMax);

        // Compute raw derivative signal
        double dRaw;
        if (first) {
            dRaw = 0.0;
        } else if (cfg.derivativeOnMeasurement) {
            // derivative on measurement: dTerm uses -dy/dt, reduces kick on step setpoint
            dRaw = -(measurement - prevMeas) / dt;
        } else {
            // derivative on error
            dRaw = (error - prevError) / dt;
        }

        // Low-pass filter derivative: 1st-order filter, stable for irregular dt
        // alpha = dt / (tau + dt)
        double alpha = dt / (cfg.dTau + dt);
        dFilt += alpha * (dRaw - dFilt);

        // Fuzzy gain scheduling
        scheduleGains(error, dFilt);

        // PID terms
        double p = kpEff * error;

        // Integrator update (trapezoidal on error) for better discrete behavior
        if (first) {
            integrator += error * dt;
        } else {
            integrator += 0.5 * (error + prevError) * dt;
        }

        // Clamp integrator state
        integrator = clamp(integrator, cfg.iMin, cfg.iMax);

        double i = kiEff * integrator;
        double d = kdEff * dFilt;

        double uUnsat = p + i + d;
        double uSat = clamp(uUnsat, cfg.outMin, cfg.outMax);

        // Anti-windup (back-calculation): drive integrator to match saturation
        if (cfg.awGain > 0.0 && kiEff != 0.0) {
            double satErr = uSat - uUnsat; // negative if we're saturating high
            integrator += (cfg.awGain * satErr / kiEff) * dt;
            integrator = clamp(integrator, cfg.iMin, cfg.iMax);
        }

        // Save state
        prevError = error;
        prevMeas = measurement;
        first = false;

        return uSat;
    }

    public double getKpEffective() { return kpEff; }
    public double getKiEffective() { return kiEff; }
    public double getKdEffective() { return kdEff; }
    public double getIntegratorState() { return integrator; }
    public double getFilteredDerivative() { return dFilt; }

    // ===== Fuzzy logic: membership + rules =====

    /**
     * Produces kpEff/kiEff/kdEff from base gains using fuzzy rules on:
     * - normalized error magnitude |e| / eScale
     * - normalized error rate de/dt / deScale  (here we feed dFilt which is already a derivative signal)
     */
    private void scheduleGains(double error, double dSignal) {
        double eN = Math.abs(error) / Math.max(1e-12, cfg.eScale);
        double deN = Math.abs(dSignal) / Math.max(1e-12, cfg.deScale);

        // Memberships for magnitude: Small, Medium, Large over [0..infty), effectively [0..2]
        double eS = tri(eN, 0.0, 0.0, 0.7);
        double eM = tri(eN, 0.3, 0.9, 1.6);
        double eL = tri(eN, 1.0, 2.0, 2.0);

        double deS = tri(deN, 0.0, 0.0, 0.7);
        double deM = tri(deN, 0.3, 0.9, 1.6);
        double deL = tri(deN, 1.0, 2.0, 2.0);

        // Rule weights (use product inference; you can switch to min if you prefer)
        double wSS = eS * deS;
        double wSM = eS * deM;
        double wSL = eS * deL;

        double wMS = eM * deS;
        double wMM = eM * deM;
        double wML = eM * deL;

        double wLS = eL * deS;
        double wLM = eL * deM;
        double wLL = eL * deL;

        double wSum = wSS + wSM + wSL + wMS + wMM + wML + wLS + wLM + wLL;
        if (wSum < 1e-12) {
            kpEff = cfg.kp; kiEff = cfg.ki; kdEff = cfg.kd;
            return;
        }

        /*
         * Gain scaling table (strong, practical defaults):
         *
         * Intuition:
         * - Large error -> raise Kp (hit it harder).
         * - Large error-rate -> raise Kd (damp/anticipate).
         * - Small error, small rate -> allow Ki to clean up steady-state.
         * - Small error but large rate -> reduce Ki (avoid integrating noise) and use more Kd.
         *
         * Entries are multipliers on base gains.
         */

        // Kp multipliers
        double kpSS = 0.7, kpSM = 0.8, kpSL = 0.9;
        double kpMS = 1.0, kpMM = 1.1, kpML = 1.2;
        double kpLS = 1.3, kpLM = 1.5, kpLL = 1.7;

        // Ki multipliers
        double kiSS = 1.6, kiSM = 1.2, kiSL = 0.7;
        double kiMS = 1.2, kiMM = 0.9, kiML = 0.5;
        double kiLS = 0.6, kiLM = 0.35, kiLL = 0.2;

        // Kd multipliers
        double kdSS = 0.5, kdSM = 0.9, kdSL = 1.4;
        double kdMS = 0.8, kdMM = 1.2, kdML = 1.6;
        double kdLS = 1.0, kdLM = 1.5, kdLL = 2.0;

        double kpMul =
                (wSS*kpSS + wSM*kpSM + wSL*kpSL +
                        wMS*kpMS + wMM*kpMM + wML*kpML +
                        wLS*kpLS + wLM*kpLM + wLL*kpLL) / wSum;

        double kiMul =
                (wSS*kiSS + wSM*kiSM + wSL*kiSL +
                        wMS*kiMS + wMM*kiMM + wML*kiML +
                        wLS*kiLS + wLM*kiLM + wLL*kiLL) / wSum;

        double kdMul =
                (wSS*kdSS + wSM*kdSM + wSL*kdSL +
                        wMS*kdMS + wMM*kdMM + wML*kdML +
                        wLS*kdLS + wLM*kdLM + wLL*kdLL) / wSum;

        kpEff = cfg.kp * kpMul;
        kiEff = cfg.ki * kiMul;
        kdEff = cfg.kd * kdMul;
    }

    /**
     * Triangular membership function.
     * tri(x, a, b, c) where:
     * - peak at b (value 1)
     * - 0 at a and c
     * Allows shoulder cases with a==b or b==c.
     */
    private static double tri(double x, double a, double b, double c) {
        if (a == b && x <= b) return 1.0;
        if (b == c && x >= b) return 1.0;
        if (x <= a || x >= c) return 0.0;
        if (x == b) return 1.0;
        if (x < b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static void validateConfig(org.firstinspires.ftc.teamcode.subsystems.FuzzyPidController.Config c) {
        if (!(c.outMax > c.outMin)) throw new IllegalArgumentException("outMax must be > outMin");
        if (!(c.dtMax > c.dtMin)) throw new IllegalArgumentException("dtMax must be > dtMin");
        if (c.dTau < 0.0) throw new IllegalArgumentException("dTau must be >= 0");
        if (c.eScale <= 0.0) throw new IllegalArgumentException("eScale must be > 0");
        if (c.deScale <= 0.0) throw new IllegalArgumentException("deScale must be > 0");
        if (c.awGain < 0.0) throw new IllegalArgumentException("awGain must be >= 0");
    }
}