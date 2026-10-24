package org.firstinspires.ftc.teamcode.swerve;

/**
 * Turns wheels to an angle by their steering motor encoders, always ending
 * with a final approach in the + direction. A wheel whose play is already on
 * the + side (its last turn ended that way) goes straight there on a + turn.
 * Otherwise, a turn that goes the - way or is shorter than
 * BACKLASH_TAKEUP_DEG first backs off past the target and comes back.
 *
 * The Through Bores' couplings have play, so what one reads depends on which
 * way its wheel last turned. Ending every turn the same way leaves the play on
 * the same side every time AbsoluteHome reads them.
 *
 * Turns never reverse mid-way; each leg stops at its target.
 *
 * Call track() once per loop, then update() for each turning wheel.
 */
public class SteerApproach {

    /**
     * Every turn ends with a + approach at least this long, degrees. Must be
     * more than the biggest Through Bore play (BL's is ~9°), or the approach
     * can end without having moved the Through Bore at all.
     */
    public static final double BACKLASH_TAKEUP_DEG = 15.0;

    /** Per wheel, FL FR BL BR: false turns that wheel straight to its target, never backing off. */
    public static final boolean[] BACK_OFF = {true, true, false, true};

    /**
     * Least power near the target. It rises by KICK_RAMP_PER_S while a wheel
     * is stuck and falls by KICK_DECAY_PER_S once it moves, so each wheel gets
     * just enough to beat friction instead of a fixed kick that overshoots.
     * Starts high: on the ground a small turn with less doesn't move at all.
     */
    private static final double KICK_START = 0.3;
    private static final double KICK_RAMP_PER_S = 3.0;
    private static final double KICK_DECAY_PER_S = 3.0;
    /** Moving slower than this counts as stuck, degrees per second. */
    private static final double STUCK_DEG_PER_S = 3.0;
    /** How often being stuck is checked, seconds (a steering tick is ~0.23°, so not every loop). */
    private static final double STUCK_CHECK_S = 0.05;
    /** A leg ends this close to its target, or past it, by the steering encoder, degrees. */
    private static final double LEG_DONE_DEG = 0.3;
    /** A leg ends after this long even if the wheel isn't there. */
    private static final double LEG_TIMEOUT_S = 3.0;

    private final SwerveRig rig;
    /** Most steering power while turning. */
    private final double maxPower;
    /** Power per degree still to go; maxPower from 30° out. */
    private final double powerPerDeg;

    /** Wheel angle by the steering encoder, counted on past ±180 so a turn never jumps, degrees. */
    private final double[] posDeg = new double[4];
    private final double[] lastRawDeg = new double[4];
    private final boolean[] turning = new boolean[4];
    private final double[] targetDeg = new double[4];
    /** The leg under way: where to, which way (+1/-1), and whether it's the final + approach. */
    private final double[] legTargetDeg = new double[4];
    private final double[] legDir = new double[4];
    private final boolean[] finalLeg = new boolean[4];
    private final double[] legTime = new double[4];
    private final double[] legStartDeg = new double[4];
    /** The wheel's last turn ended with a full + approach, so its Through Bore's play is on the + side. */
    private final boolean[] playOnPlusSide = new boolean[4];
    private final boolean[] plusAtLegStart = new boolean[4];
    private final double[] kick = new double[4];
    private final double[] stuckClock = new double[4];
    private final double[] stuckFromDeg = new double[4];

    public SteerApproach(SwerveRig rig, double maxPower) {
        this.rig = rig;
        this.maxPower = maxPower;
        this.powerPerDeg = maxPower / 30.0;
        for (int i = 0; i < 4; i++) {
            resetPosition(i);
        }
    }

    /** Follow the steering encoders. Call once per loop, before update(). */
    public void track() {
        for (int i = 0; i < 4; i++) {
            double raw = rig.getHomeErrorDegrees(i);
            posDeg[i] += wrap(raw - lastRawDeg[i]);
            lastRawDeg[i] = raw;
        }
    }

    /** Start counting from the steering encoder's angle as it is now, e.g. after it was seeded. */
    public void resetPosition(int i) {
        lastRawDeg[i] = rig.getHomeErrorDegrees(i);
        posDeg[i] = lastRawDeg[i];
    }

    /**
     * Start turning wheel i to target (same count as the steering encoder).
     * Goes straight there if the turn is + and the play is already known to
     * be on the + side (or the turn is long enough to put it there); only
     * otherwise backs off BACKLASH_TAKEUP_DEG past the target first. A wheel
     * with BACK_OFF false always goes straight there.
     */
    public void start(int i, double target) {
        targetDeg[i] = target;
        turning[i] = true;
        double move = target - posDeg[i];
        if (!BACK_OFF[i] || move >= BACKLASH_TAKEUP_DEG || (playOnPlusSide[i] && move >= 0.0)) {
            startLeg(i, target, true);
        } else {
            startLeg(i, target - BACKLASH_TAKEUP_DEG, false);
        }
    }

    public boolean isTurning(int i) {
        return turning[i];
    }

    /** True on the final + approach, false while backing off first. */
    public boolean isOnFinalLeg(int i) {
        return finalLeg[i];
    }

    /** Stop turning wheel i at once. */
    public void cancel(int i) {
        turning[i] = false;
        rig.setSteerPowerDirect(i, 0.0);
    }

    private void startLeg(int i, double target, boolean last) {
        legTargetDeg[i] = target;
        legDir[i] = Math.signum(target - posDeg[i]);
        finalLeg[i] = last;
        legTime[i] = 0.0;
        legStartDeg[i] = posDeg[i];
        // Backing off (-) moves the play over to the - side.
        plusAtLegStart[i] = last && playOnPlusSide[i];
        if (!last) {
            playOnPlusSide[i] = false;
        }
        kick[i] = KICK_START;
        stuckClock[i] = 0.0;
        stuckFromDeg[i] = posDeg[i];
    }

    /** Power wheel i toward its target; isTurning() goes false once it's there. */
    public void update(int i, double dt) {
        if (!turning[i]) {
            return;
        }
        legTime[i] += dt;
        double togo = legTargetDeg[i] - posDeg[i];
        // Never pushed back the other way: that would spend the play again.
        boolean timedOut = legTime[i] > LEG_TIMEOUT_S;
        if (legDir[i] == 0.0 || togo * legDir[i] <= LEG_DONE_DEG || timedOut) {
            rig.setSteerPowerDirect(i, 0.0);
            if (finalLeg[i]) {
                turning[i] = false;
                // The play is on the + side if it already was, or this
                // approach was long enough to push it there.
                double moved = posDeg[i] - legStartDeg[i];
                playOnPlusSide[i] = !timedOut && moved >= 0.0
                        && (plusAtLegStart[i] || moved >= BACKLASH_TAKEUP_DEG - LEG_DONE_DEG);
            } else {
                startLeg(i, targetDeg[i], true);
            }
            return;
        }

        stuckClock[i] += dt;
        if (stuckClock[i] >= STUCK_CHECK_S) {
            double speed = Math.abs(posDeg[i] - stuckFromDeg[i]) / stuckClock[i];
            if (speed < STUCK_DEG_PER_S) {
                kick[i] = Math.min(maxPower, kick[i] + KICK_RAMP_PER_S * stuckClock[i]);
            } else {
                kick[i] = Math.max(KICK_START, kick[i] - KICK_DECAY_PER_S * stuckClock[i]);
            }
            stuckClock[i] = 0.0;
            stuckFromDeg[i] = posDeg[i];
        }
        double p = PIDController.clamp(powerPerDeg * Math.abs(togo), kick[i], maxPower);
        rig.setSteerPowerDirect(i, legDir[i] * p);
    }

    private static double wrap(double deg) {
        return Math.toDegrees(SwerveHardware.normalize(Math.toRadians(deg)));
    }
}
