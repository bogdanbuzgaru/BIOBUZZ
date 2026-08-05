package org.firstinspires.ftc.teamcode.pid;

import java.util.function.Function;

public class PIDF extends PID {

    private Function<Double, Double> feedforward;

    public PIDF(double kp, double ki, double kd, Function<Double, Double> feedforward) {
        super(kp, ki, kd);
        this.feedforward = feedforward;
    }

    protected void setFeedforward(Function<Double, Double> feedforward) {
        this.feedforward = feedforward;
    }

    @Override
    public double feed(double value, double time) {
        return super.feed(value, time) + feedforward.apply(value);
    }
}