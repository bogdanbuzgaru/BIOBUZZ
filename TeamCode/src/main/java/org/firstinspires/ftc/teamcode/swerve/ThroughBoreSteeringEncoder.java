package org.firstinspires.ftc.teamcode.swerve;

/**
 * Azimuth sensor built on the REV Through Bore Encoder's QUADRATURE output,
 * plugged into a motor encoder port on the Control or Expansion Hub.
 *
 * Why not the absolute output: REV's own datasheet states that the FTC Control
 * System (Control Hub and Expansion Hub) only supports incremental encoder input
 * through the motor encoder ports, and that absolute pulse input is not
 * supported. The absolute signal is a duty-cycle pulse train, not a voltage and
 * not I2C, and nothing on the hub can decode it. Feeding it to an analog port
 * gives you the instantaneous high or low level, not the pulse width.
 *
 * So the plan is: read the Through Bore as quadrature (8192 counts/rev, which is
 * about 0.044 degrees of azimuth resolution, far better than the steering
 * motor's own 537.7 counts/rev), and pay for it with a homing step at startup.
 *
 * Wiring, per module:
 *   Through Bore 6-pin  --(REV-31-1815 6-pin to 4-pin JST-PH)-->  encoder port
 *                                                                 of the 312 RPM
 *                                                                 steering motor
 *   Mode switch on the encoder body set to 'A' (ABI output mode).
 *
 * You then read it with steerMotor.getCurrentPosition(). That motor's own
 * internal encoder becomes unavailable, which costs you nothing here: the
 * Through Bore is mounted on the azimuth axis itself, so it is immune to any
 * backlash in the steering gearbox.
 */
public class ThroughBoreSteeringEncoder implements SwerveHardware.AngleSensor {

    /** REV Through Bore quadrature resolution, counts per revolution. */
    public static final double COUNTS_PER_REV = 8192.0;

    private final SwerveHardware.DoubleSource rawTicks;
    private final double radiansPerTick;
    private final boolean inverted;

    private double offsetTicks = 0.0;
    private boolean homed = false;

    /**
     * @param rawTicks   e.g. steerMotor::getCurrentPosition
     * @param gearRatio  encoder revolutions per full 360 degree rotation of the
     *                   module. Use 1.0 if the encoder sits directly on the
     *                   azimuth axis, which is the normal and much better case.
     * @param inverted   true if turning the module counter-clockwise makes the
     *                   tick count go DOWN
     */
    public ThroughBoreSteeringEncoder(SwerveHardware.DoubleSource rawTicks,
                                      double gearRatio,
                                      boolean inverted) {
        this.rawTicks = rawTicks;
        this.radiansPerTick = (2.0 * Math.PI) / (COUNTS_PER_REV * gearRatio);
        this.inverted = inverted;
    }

    @Override
    public double getAngleRadians() {
        double ticks = rawTicks.get() - offsetTicks;
        double angle = ticks * radiansPerTick;
        if (inverted) {
            angle = -angle;
        }
        return SwerveHardware.normalize(angle);
    }

    /** Wrapped angle in degrees, for telemetry. */
    public double getAngleDegrees() {
        return Math.toDegrees(getAngleRadians());
    }

    /**
     * Continuous, unwrapped angle. Useful for telemetry while homing, because it
     * lets you see that the module really did turn 90 degrees rather than
     * wrapping around.
     */
    public double getContinuousAngleRadians() {
        double angle = (rawTicks.get() - offsetTicks) * radiansPerTick;
        return inverted ? -angle : angle;
    }

    public double getContinuousAngleDegrees() {
        return Math.toDegrees(getContinuousAngleRadians());
    }

    /** Degrees of azimuth per encoder tick. 8192 counts/rev gives 0.0439. */
    public double getDegreesPerTick() {
        return Math.toDegrees(radiansPerTick);
    }

    /**
     * Call with the wheel physically pointed straight forward. Everything
     * downstream depends on this being right.
     */
    public void home() {
        setCurrentAngle(0.0);
    }

    /** Declare the module's true current angle, in radians. */
    public void setCurrentAngle(double trueAngleRadians) {
        double a = inverted ? -trueAngleRadians : trueAngleRadians;
        offsetTicks = rawTicks.get() - (a / radiansPerTick);
        homed = true;
    }

    /**
     * Seed from a known offset measured once and written down, rather than from
     * a physical alignment. Use this if you fit a real analog absolute sensor
     * later: read it at init, pass the value here, and the homing ritual goes
     * away while you keep the Through Bore's resolution.
     */
    public void seedFrom(SwerveHardware.AngleSensor absoluteSensor) {
        setCurrentAngle(absoluteSensor.getAngleRadians());
    }

    public boolean isHomed() {
        return homed;
    }

    public double getRawTicks() {
        return rawTicks.get();
    }

    // ------------------------------------------------------------------

    /**
     * Startup homing helper. Keep the robot disabled, turn all four wheels
     * forward by hand (use the flat of the module plate or a printed alignment
     * jig, not eyeballing), then call homeAll().
     *
     * Do this every power cycle. If you skip it, the modules believe whatever
     * angle they happened to be left at, and 50 kg will go somewhere you did not
     * ask for.
     */
    public static class HomingRoutine {
        private final ThroughBoreSteeringEncoder[] encoders;
        private final SwerveModule[] modules;

        public HomingRoutine(ThroughBoreSteeringEncoder[] encoders, SwerveModule[] modules) {
            this.encoders = encoders;
            this.modules = modules;
        }

        /** Motors off, so the wheels can be turned by hand. */
        public void relax() {
            for (SwerveModule m : modules) {
                m.stop();
            }
        }

        /** Call once the wheels are physically aligned forward. */
        public void homeAll() {
            for (ThroughBoreSteeringEncoder e : encoders) {
                e.home();
            }
            for (SwerveModule m : modules) {
                m.reset();
            }
        }

        /**
         * Home against something other than straight-forward.
         *
         * Use this if aligning to a repeatable mechanical feature is easier than
         * eyeballing forward: park every module against a jig, a chassis rail or
         * a hard stop, then pass the angle from that feature to forward for each
         * module. SwerveCalibration measures those numbers for you.
         *
         * @param trueAnglesRadians one per module, in the same order as the array
         *                          this routine was constructed with
         */
        public void homeAll(double[] trueAnglesRadians) {
            if (trueAnglesRadians.length != encoders.length) {
                throw new IllegalArgumentException("one angle per module required");
            }
            for (int i = 0; i < encoders.length; i++) {
                encoders[i].setCurrentAngle(trueAnglesRadians[i]);
            }
            for (SwerveModule m : modules) {
                m.reset();
            }
        }

        public boolean allHomed() {
            for (ThroughBoreSteeringEncoder e : encoders) {
                if (!e.isHomed()) {
                    return false;
                }
            }
            return true;
        }
    }
}
