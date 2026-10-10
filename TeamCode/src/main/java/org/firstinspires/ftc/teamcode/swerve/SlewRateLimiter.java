package org.firstinspires.ftc.teamcode.swerve;
/**
 * Limits how fast a scalar command can change, in units per second.
 *
 * On a 50 kg robot this is doing real work: the driver's thumb can move a stick
 * from 0 to 1 in about 50 ms, which is an acceleration request the drivetrain
 * cannot meet. The motors just saturate, the wheels slip, and the modules fight
 * each other. Limiting the command to something the robot can physically do
 * means the closed loop stays inside its linear range and your PID gains stay
 * valid.
 */
public class SlewRateLimiter {

    private double rateLimit;
    private double value;
    private boolean initialized = false;

    public SlewRateLimiter(double unitsPerSecond) {
        this(unitsPerSecond, 0.0);
    }

    public SlewRateLimiter(double unitsPerSecond, double initialValue) {
        this.rateLimit = Math.abs(unitsPerSecond);
        this.value = initialValue;
    }

    public void setRateLimit(double unitsPerSecond) {
        this.rateLimit = Math.abs(unitsPerSecond);
    }

    public double calculate(double input, double dt) {
        if (!initialized) {
            value = input;
            initialized = true;
            return value;
        }
        double maxDelta = rateLimit * Math.max(dt, 0.0);
        double delta = input - value;
        if (delta > maxDelta) {
            delta = maxDelta;
        } else if (delta < -maxDelta) {
            delta = -maxDelta;
        }
        value += delta;
        return value;
    }

    public void reset(double newValue) {
        this.value = newValue;
        this.initialized = true;
    }

    public double get() {
        return value;
    }
}
