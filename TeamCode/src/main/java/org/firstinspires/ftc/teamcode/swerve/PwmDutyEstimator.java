package org.firstinspires.ftc.teamcode.swerve;

/**
 * Estimates the angle of a REV Through Bore from its ABSOLUTE (white wire)
 * output read on a plain digital port.
 *
 * The absolute output is a pulse repeating about every 1025 us whose HIGH time
 * is the angle: ~1 us at 0 degrees, ~1024 us at 360. A digital port cannot time
 * that pulse; it only says high or low at the instant it is read. But if you
 * read it many times at moments unrelated to the pulse, the fraction of reads
 * that come back high is the fraction of time the signal is high, which is the
 * angle / 360.
 *
 * That only works while the wheel is still, and it is noisy: the standard error
 * is sqrt(d(1 - d) / n) of a turn, so 400 samples is up to about +-9 degrees and
 * 4000 samples about +-3. Use it to find where the wheel points at startup, not
 * to steer with.
 *
 * Near 0 / 360 degrees the encoder flips between its shortest and longest
 * pulse, so a wheel parked right there can read anywhere. getStdErrorDegrees()
 * cannot see that; compare against the wheel itself the first few times.
 */
public class PwmDutyEstimator {

    private final boolean[] ring;
    private int next = 0;
    private int count = 0;
    private int highs = 0;

    /** @param capacity how many of the most recent samples to average */
    public PwmDutyEstimator(int capacity) {
        ring = new boolean[capacity];
    }

    public void addSample(boolean high) {
        if (count == ring.length) {
            if (ring[next]) {
                highs--;
            }
        } else {
            count++;
        }
        ring[next] = high;
        if (high) {
            highs++;
        }
        next = (next + 1) % ring.length;
    }

    public void reset() {
        next = 0;
        count = 0;
        highs = 0;
    }

    public int getSampleCount() {
        return count;
    }

    /** Fraction of samples that read high, 0..1. */
    public double getDuty() {
        return count == 0 ? 0.0 : (double) highs / count;
    }

    /** Encoder angle in degrees, 0..360, as the encoder itself sees it. */
    public double getAngleDegrees() {
        return getDuty() * 360.0;
    }

    /** One standard error of getAngleDegrees(), from sampling noise alone. */
    public double getStdErrorDegrees() {
        if (count == 0) {
            return 180.0;
        }
        double d = getDuty();
        // Never report better than one sample's worth of resolution.
        double variance = Math.max(d * (1.0 - d), 1.0 / count) / count;
        return Math.sqrt(variance) * 360.0;
    }
}
