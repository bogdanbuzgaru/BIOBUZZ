package org.firstinspires.ftc.teamcode.pid;

public class PID {
    protected double kp, ki, kd; // Constants which we choose experimentally
    protected double lastError;
    protected double lastTime;
    protected double integral;
    protected double local_target;

    protected double integralLimit;

    public void setIntegralLimitFactor(double factor) {
        this.integralLimit = factor;
    }

    public double getLocal_target() {
        return local_target;
    }

    public void setLocal_target(double local_target) {
        this.local_target = local_target;
    }

    /**
     * Initialization
     * @param kp constant for the proportional
     * @param ki constant for the integral
     * @param kd constant for the derivative
     */
    public PID(double kp, double ki, double kd) {
        this.ki = ki;
        this.kp = kp;
        this.kd = kd;
        local_target = 0;
        reset();
    }

    /**
     * setting the target
     * @param target the target for the PID controller
     */
    public void setTarget(double target) {
        local_target = target;
    }

    /**
     * The derivative measures the error in the elapsed time
     * The integral approximates the area of error (how far the robot has deviated)
     * @param value input value
     * @param time the elapsed time
     * @return returning the correction
     */
    public double feed(double value, double time) {
        double error = local_target - value;
        double derivative = (error - lastError) / (time - lastTime);
        integral += (error * time);
        if (integral != 0) {
            double limit = Math.abs(integralLimit / integral);
            if (integral < -limit) {
                integral = - limit;
            }
            if (integral > limit) {
                integral = limit;
            }
        }

        lastError = error;
        lastTime = time;
        return error * kp + integral * ki + derivative * kd; //The equation
    }

    public void reset() {
        lastError = 0;
        lastTime = 0;
        integral = 0;
    }

    public double kp() {
        return kp;
    }

    public void kp(double kp) {
        this.kp = kp;
    }

    public double ki() {
        return ki;
    }

    public void ki(double ki) {
        this.ki = ki;
    }

    public double kd() {
        return kd;
    }

    public void kd(double kd) {
        this.kd = kd;
    }
}
