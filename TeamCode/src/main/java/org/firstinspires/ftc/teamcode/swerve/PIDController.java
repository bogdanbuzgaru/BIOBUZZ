package org.firstinspires.ftc.teamcode.swerve;
/**
 * General-purpose PID controller built for drivetrain work.
 *
 * Differences from a textbook PID, and why they matter on a heavy robot:
 *
 *  - Derivative on measurement, not on error. If you compute D from the error,
 *    a step change in the setpoint produces an infinite spike ("derivative kick").
 *    On a 50 kg robot that spike becomes a current spike.
 *
 *  - Low-pass filtered derivative. Quadrature encoders are quantized; raw
 *    dError/dt is mostly noise at small dt. The filter is what makes kD usable.
 *
 *  - Conditional integration (anti-windup). The integrator only accumulates when
 *    the output is not already saturated in the direction the error is pushing.
 *    Without this, a stalled steering module winds the integrator up and the wheel
 *    slams past its target when it breaks free.
 *
 *  - Optional continuous input, for angles. Tell it the input wraps at -pi..pi and
 *    an error of 350 degrees is treated as -10 degrees.
 *
 * Units are whatever you feed it. Output is clamped to [minOutput, maxOutput],
 * which defaults to [-1, 1] so it can drive a motor directly.
 */
public class PIDController {

    private double kP;
    private double kI;
    private double kD;

    // ---- integrator state ----
    private double integral = 0.0;
    private double iZone = Double.POSITIVE_INFINITY;
    private double maxIntegralContribution = Double.POSITIVE_INFINITY;

    // ---- derivative state ----
    private double lastMeasurement = 0.0;
    private double filteredDerivative = 0.0;
    /** 1.0 = no filtering (noisy), 0.1 = heavy filtering (laggy). 0.3 is a good start. */
    private double derivativeAlpha = 0.3;
    private boolean firstRun = true;

    // ---- output clamp ----
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    // ---- continuous (angular) input ----
    private boolean continuous = false;
    private double inputMin = 0.0;
    private double inputRange = 0.0;

    // ---- tolerances ----
    private double positionTolerance = 0.0;
    private double velocityTolerance = Double.POSITIVE_INFINITY;

    // ---- telemetry ----
    private double error = 0.0;
    private double derivative = 0.0;
    private double setpoint = 0.0;
    private double lastOutput = 0.0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // ------------------------------------------------------------------
    // configuration
    // ------------------------------------------------------------------

    public PIDController setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        return this;
    }

    /**
     * Treat the input as an angle that wraps. For radians use (-Math.PI, Math.PI).
     */
    public PIDController enableContinuousInput(double minInput, double maxInput) {
        this.continuous = true;
        this.inputMin = minInput;
        this.inputRange = maxInput - minInput;
        return this;
    }

    public PIDController disableContinuousInput() {
        this.continuous = false;
        return this;
    }

    public PIDController setOutputRange(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
        return this;
    }

    /**
     * Only integrate when |error| is below this. Keeps the integrator from
     * charging up during large moves, where P already saturates the output.
     */
    public PIDController setIZone(double iZone) {
        this.iZone = Math.abs(iZone);
        return this;
    }

    /** Hard cap on how much of the output the I term is allowed to produce. */
    public PIDController setMaxIntegralContribution(double max) {
        this.maxIntegralContribution = Math.abs(max);
        return this;
    }

    /** 1.0 = unfiltered derivative, smaller = smoother but more phase lag. */
    public PIDController setDerivativeFilter(double alpha) {
        this.derivativeAlpha = clamp(alpha, 0.01, 1.0);
        return this;
    }

    public PIDController setTolerance(double positionTolerance) {
        return setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public PIDController setTolerance(double positionTolerance, double velocityTolerance) {
        this.positionTolerance = Math.abs(positionTolerance);
        this.velocityTolerance = Math.abs(velocityTolerance);
        return this;
    }

    // ------------------------------------------------------------------
    // execution
    // ------------------------------------------------------------------

    /**
     * @param setpoint    where you want to be
     * @param measurement where you are
     * @param dt          seconds since the previous call
     */
    public double calculate(double setpoint, double measurement, double dt) {
        if (dt <= 0) {
            return lastOutput;
        }
        // Guard against absurdly small loop times blowing up the derivative.
        dt = Math.max(dt, 1e-4);

        this.setpoint = setpoint;

        if (firstRun) {
            lastMeasurement = measurement;
            filteredDerivative = 0.0;
            firstRun = false;
        }

        double err = setpoint - measurement;
        if (continuous) {
            err = wrap(err);
        }
        this.error = err;

        // --- D: computed from the measurement so setpoint steps do not spike it ---
        double measurementDelta = measurement - lastMeasurement;
        if (continuous) {
            measurementDelta = wrap(measurementDelta);
        }
        double rawDerivative = -measurementDelta / dt;
        filteredDerivative += derivativeAlpha * (rawDerivative - filteredDerivative);
        this.derivative = filteredDerivative;
        lastMeasurement = measurement;

        double p = kP * err;
        double d = kD * filteredDerivative;

        // --- I: conditional integration ---
        double provisional = p + d + kI * integral;
        if (Math.abs(err) <= iZone) {
            boolean pushingIntoUpperSaturation = provisional >= maxOutput && err > 0;
            boolean pushingIntoLowerSaturation = provisional <= minOutput && err < 0;
            if (!pushingIntoUpperSaturation && !pushingIntoLowerSaturation) {
                integral += err * dt;
                if (kI != 0.0 && maxIntegralContribution != Double.POSITIVE_INFINITY) {
                    double limit = maxIntegralContribution / Math.abs(kI);
                    integral = clamp(integral, -limit, limit);
                }
            }
        } else {
            integral = 0.0;
        }

        double output = clamp(p + kI * integral + d, minOutput, maxOutput);
        lastOutput = output;
        return output;
    }

    /**
     * Clear all internal state. Call this whenever the loop has been paused, the
     * mechanism has been moved by hand, or you switch control modes. Stale
     * integrator and derivative state after a pause is a very common source of
     * "it lurched on enable".
     */
    public void reset() {
        integral = 0.0;
        filteredDerivative = 0.0;
        derivative = 0.0;
        error = 0.0;
        lastOutput = 0.0;
        firstRun = true;
    }

    public boolean atSetpoint() {
        return Math.abs(error) <= positionTolerance
                && Math.abs(derivative) <= velocityTolerance;
    }

    public double getError() {
        return error;
    }

    public double getDerivative() {
        return derivative;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getIntegral() {
        return integral;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------

    private double wrap(double value) {
        if (inputRange <= 0) {
            return value;
        }
        double wrapped = (value - inputMin) % inputRange;
        if (wrapped < 0) {
            wrapped += inputRange;
        }
        wrapped += inputMin;
        // fold into the shorter half of the range
        if (wrapped > inputRange / 2.0) {
            wrapped -= inputRange;
        } else if (wrapped < -inputRange / 2.0) {
            wrapped += inputRange;
        }
        return wrapped;
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}