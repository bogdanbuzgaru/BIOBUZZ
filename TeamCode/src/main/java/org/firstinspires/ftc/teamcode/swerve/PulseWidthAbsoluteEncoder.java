package org.firstinspires.ftc.teamcode.swerve;

/**
 * Absolute azimuth from the REV Through Bore's pulse-width output, decoded by an
 * external I2C module such as the OctoQuad.
 *
 * The Control Hub itself still cannot read this signal. What the OctoQuad does
 * is decode the pulse train on its own microcontroller and expose the measured
 * width in microseconds over I2C, which the hub reads in a single bulk
 * transaction. That is why this class takes microseconds rather than talking to
 * a port directly: hand it whatever your decoder's driver returns.
 *
 * Wiring per module:
 *   Through Bore 6-pin --(REV-11-1817, 3-pin PWM lead)--> OctoQuad channel
 *   OctoQuad --(I2C)--> any I2C port on the Control Hub
 *
 * Four modules uses four of the eight channels, so the remaining four are free
 * for odometry pods or anything else.
 *
 * Best arrangement if you have both: feed this into
 * ThroughBoreSteeringEncoder.seedFrom() once at init. You get the absolute
 * reading's zero knowledge with the quadrature output's 8192-count resolution,
 * and no homing step at all.
 */
public class PulseWidthAbsoluteEncoder implements SwerveHardware.AngleSensor {

    /**
     * Through Bore V2 (MPS MA600): 3.884 us at 0 degrees, 998.06 us at 360,
     * 12-bit resolution. V1 (Broadcom AEAT-8800) is 10-bit over roughly
     * 1 to 1024 us. Check the datasheet revision that matches your part.
     */
    public static final double V2_MIN_PULSE_US = 3.884;
    public static final double V2_MAX_PULSE_US = 998.06;

    private final SwerveHardware.DoubleSource pulseWidthMicros;
    private final double minPulseUs;
    private final double spanUs;
    private final double offsetRadians;
    private final boolean inverted;

    /** Last good reading, held if the decoder returns something impossible. */
    private double lastGoodAngle = 0.0;
    private int faultCount = 0;

    /**
     * @param pulseWidthMicros  measured pulse width, from your decoder's driver
     * @param minPulseUs        pulse width at 0 degrees
     * @param maxPulseUs        pulse width at 360 degrees
     * @param offsetRadians     the angle this reports when the wheel points
     *                          straight forward; measure once per module and
     *                          write it down as a named constant
     * @param inverted          true if CCW module rotation makes the reading fall
     */
    public PulseWidthAbsoluteEncoder(SwerveHardware.DoubleSource pulseWidthMicros,
                                     double minPulseUs,
                                     double maxPulseUs,
                                     double offsetRadians,
                                     boolean inverted) {
        this.pulseWidthMicros = pulseWidthMicros;
        this.minPulseUs = minPulseUs;
        this.spanUs = maxPulseUs - minPulseUs;
        this.offsetRadians = offsetRadians;
        this.inverted = inverted;
    }

    /** Convenience constructor for a Through Bore V2. */
    public static PulseWidthAbsoluteEncoder throughBoreV2(
            SwerveHardware.DoubleSource pulseWidthMicros,
            double offsetRadians,
            boolean inverted) {
        return new PulseWidthAbsoluteEncoder(
                pulseWidthMicros, V2_MIN_PULSE_US, V2_MAX_PULSE_US, offsetRadians, inverted);
    }

    @Override
    public double getAngleRadians() {
        double us = pulseWidthMicros.get();

        // A dropped or corrupted frame reads as 0 or as something far outside
        // the valid span. Holding the previous angle is much safer than letting
        // a module snap to a garbage target under 50 kg of load.
        if (us < minPulseUs - 5.0 || us > minPulseUs + spanUs + 5.0) {
            faultCount++;
            return lastGoodAngle;
        }

        double fraction = (us - minPulseUs) / spanUs;
        double angle = fraction * 2.0 * Math.PI;
        if (inverted) {
            angle = -angle;
        }
        lastGoodAngle = SwerveHardware.normalize(angle - offsetRadians);
        return lastGoodAngle;
    }

    /**
     * Number of rejected readings since startup. Watch this during bring-up: a
     * steadily climbing count means the PWM lead is picking up motor noise, and
     * the fix is routing, not software.
     */
    public int getFaultCount() {
        return faultCount;
    }

    public void resetFaultCount() {
        faultCount = 0;
    }

    /**
     * Run this once per module with the wheel physically pointed forward, print
     * the result, and paste it into your constants as offsetRadians. Construct
     * the sensor with offsetRadians = 0 while you do this.
     */
    public double measureOffsetRadians() {
        return getAngleRadians();
    }
}
