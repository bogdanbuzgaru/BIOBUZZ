package org.firstinspires.ftc.teamcode.pid;
public class PIDFLinear extends PIDF {

    private double ka, kb, k0;

    public PIDFLinear(double kp, double ki, double kd, double ka, double kb, double k0) {
        super(kp, ki, kd, null);
        this.ka = ka;
        this.kb = kb;
        this.k0 = k0;
        setFeedforward(pos -> (pos - this.k0()) * this.ka() + this.kb());
    }

    public double ka() {
        return ka;
    }

    public void ka(double ka) {
        this.ka = ka;
    }

    public double kb() {
        return kb;
    }

    public void kb(double kb) {
        this.kb = kb;
    }

    public double k0() {
        return k0;
    }

    public void k0(double k0) {
        this.k0 = k0;
    }
}
