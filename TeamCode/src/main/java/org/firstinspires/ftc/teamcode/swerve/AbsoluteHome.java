package org.firstinspires.ftc.teamcode.swerve;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turns every wheel to its FORWARD position during INIT. Call update() once
 * per INIT loop, after rig.sampleEncoders().
 *
 * FORWARD is where the wheels were when TRIANGLE was pressed in
 * OpSwerveCalibration (the saved Through Bore offsets): there each wheel's
 * REV Through Bore reads 0°. The Through Bores decide where the wheels are;
 * the steering motors' own encoders carry out the turns, because a Through
 * Bore on a digital port can only be read as a slow average while the wheel
 * stands still.
 *
 * A Through Bore reading gets more accurate the longer it averages, and the
 * whole thing has to finish inside TIME_LIMIT_S, so:
 *   1. ROUGH reading   quick: a wheel far from forward needs only a rough
 *                      angle to get close
 *   2. TURN            every wheel to forward on its steering encoder
 *   3. PRECISE reading until it is good to PRECISE_ERROR_DEG, or until
 *                      MEASURE_DEADLINE_S, whichever comes first
 *   4. TURN            the small correction that reading shows, slowly, so
 *                      the wheel stops within TURN_DONE_DEG
 *   5. If time is left, steps 3 and 4 repeat as a check. Each new reading is
 *      combined with what was already known (weighted by how exact each one
 *      is), so a short check reading can only improve the result.
 * If a reading finds a wheel more than RECHECK_ABOVE_DEG off (it didn't turn
 * properly), it stops early and the wheels turn again before measuring on.
 *
 * Tuning:
 *   - Telemetry shows samples/s and the expected accuracy at the end. More
 *     samples/s means more accuracy in the same time.
 *   - A wheel creeps to a stop short of 0° on the ground: raise KICK_RAMP_PER_S.
 *   - A wheel overshoots and swings back: lower POWER_PER_DEG or KICK_START.
 */
public class AbsoluteHome {

    // ---------------- time ----------------

    /** Homing never takes longer than this, seconds. */
    private static final double TIME_LIMIT_S = 28.0;
    /** A precise reading stops here, leaving time for the last correction. */
    private static final double MEASURE_DEADLINE_S = 24.0;
    /** Start a check reading only if at least this long is left before the deadline. */
    private static final double CHECK_MIN_S = 4.0;

    // ---------------- readings ----------------

    /** A precise reading stops once every wheel is known to this (one standard error), degrees. */
    private static final double PRECISE_ERROR_DEG = 0.75;
    /** The rough reading may be off by this fraction of the distance to forward... */
    private static final double ROUGH_FRACTION = 0.2;
    /** ...but needn't be better than this. */
    private static final double ROUGH_MIN_ERROR_DEG = 5.0;
    /** A wheel further off than this needs turning before measuring precisely... */
    private static final double RECHECK_ABOVE_DEG = 15.0;
    /** ...which a reading this exact can already tell. */
    private static final double RECHECK_KNOWN_DEG = 3.0;
    /** While measuring, sample the Through Bores this long per loop, ms. */
    private static final double SAMPLE_BURST_MS = 25.0;
    /** Fewest samples in a reading. */
    private static final int MIN_SAMPLES = 300;
    /** Time for the wheels to stop after turning before measuring. */
    private static final double SETTLE_S = 0.3;
    /** Error a turn adds to what's known (steering gear backlash), degrees. */
    private static final double TURN_ERROR_DEG = 1.0;

    // ---------------- turning ----------------

    /** Most steering power while turning: full power, to beat friction on the ground. */
    private static final double MAX_POWER = 1.0;
    /** Power per degree still to go; MAX_POWER from 30° out. */
    private static final double POWER_PER_DEG = MAX_POWER / 30.0;
    /**
     * Least power near the target. It rises by KICK_RAMP_PER_S while a wheel
     * is stuck and falls back once it moves, so each wheel gets just enough
     * to beat friction instead of a fixed kick that overshoots.
     */
    private static final double KICK_START = 0.08;
    private static final double KICK_RAMP_PER_S = 1.2;
    /** Less movement than this in one loop counts as stuck, degrees (a steering tick is ~0.23°). */
    private static final double STUCK_DEG = 0.1;
    /** The turn is done when every wheel is this close by its steering encoder... */
    private static final double TURN_DONE_DEG = 0.5;
    /** ...and has stayed there this long. */
    private static final double TURN_HOLD_S = 0.15;
    /** Stop turning after this long even if a wheel isn't there. */
    private static final double TURN_TIMEOUT_S = 2.5;
    /** Give up after this many readings. */
    private static final int MAX_READINGS = 8;

    private enum State { MEASURE, TURN, SETTLE, DONE }

    private final SwerveRig rig;
    private State state = State.MEASURE;
    private double stateTime = 0.0;
    private double totalTime = 0.0;
    private int readings = 0;
    /** The turn under way is a small correction after a precise reading. */
    private boolean correction = false;
    private double holdTime = 0.0;
    private String result = "";

    private int samplesCounted = 0;
    private double rateClock = 0.0;
    private double samplesPerSecond = 0.0;

    private final double[] errorDeg = new double[4];
    private final boolean[] measured = new boolean[4];
    /** One standard error of where each wheel is believed to be, degrees. */
    private final double[] knownDeg = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
            Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
    /** Through Bore angle at the last reading, to see which way the wheel really went. */
    private final double[] lastReadingDeg = new double[4];
    private final boolean[] wrongWay = new boolean[4];
    private final double[] kick = new double[4];
    private final double[] lastTogo = new double[4];

    public AbsoluteHome(SwerveRig rig) {
        this.rig = rig;
        rig.restartSeedAverage();
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    /** True while the steering motors are turning the wheels. */
    public boolean isTurning() {
        return state == State.TURN;
    }

    public void update(double dt) {
        stateTime += dt;
        if (state != State.DONE) {
            totalTime += dt;
        }
        switch (state) {
            case MEASURE:
                allPowerOff();
                countSamples(rig.sampleEncodersFor(SAMPLE_BURST_MS), dt);
                if (readingsReady()) {
                    useReadings();
                }
                break;

            case TURN:
                boolean allThere = true;
                for (int i = 0; i < 4; i++) {
                    if (!usable(i)) {
                        rig.setSteerPowerDirect(i, 0.0);
                        continue;
                    }
                    // By the steering encoder, seeded from the readings.
                    double togo = -rig.getHomeErrorDegrees(i);
                    double moved = Math.abs(togo - lastTogo[i]);
                    lastTogo[i] = togo;
                    if (Math.abs(togo) <= TURN_DONE_DEG) {
                        rig.setSteerPowerDirect(i, 0.0);
                        kick[i] = KICK_START;
                        continue;
                    }
                    allThere = false;
                    if (moved < STUCK_DEG) {
                        kick[i] = Math.min(MAX_POWER, kick[i] + KICK_RAMP_PER_S * dt);
                    } else {
                        kick[i] = Math.max(KICK_START, kick[i] - KICK_RAMP_PER_S * dt);
                    }
                    double p = PIDController.clamp(POWER_PER_DEG * Math.abs(togo), kick[i], MAX_POWER);
                    rig.setSteerPowerDirect(i, Math.copySign(p, togo));
                }
                holdTime = allThere ? holdTime + dt : 0.0;
                if (holdTime >= TURN_HOLD_S || stateTime > TURN_TIMEOUT_S || totalTime > TIME_LIMIT_S) {
                    turnEnded();
                }
                break;

            case SETTLE:
                allPowerOff();
                if (stateTime > SETTLE_S) {
                    rig.restartSeedAverage();
                    enter(State.MEASURE);
                }
                break;

            case DONE:
                allPowerOff();
                break;
        }
    }

    private void countSamples(int n, double dt) {
        samplesCounted += n;
        rateClock += dt;
        if (rateClock >= 0.5) {
            samplesPerSecond = samplesCounted / rateClock;
            samplesCounted = 0;
            rateClock = 0.0;
        }
    }

    /** True for a wheel whose Through Bore can be used. */
    private boolean usable(int i) {
        return rig.getSeedSampleCount(i) < MIN_SAMPLES || rig.isThroughBorePulsing(i);
    }

    private boolean isRoughReading() {
        return readings == 0;
    }

    /** Every usable wheel's reading is accurate enough for this step, or time is up. */
    private boolean readingsReady() {
        boolean allPrecise = true;
        boolean allKnown = true;
        boolean farOff = false;
        for (int i = 0; i < 4; i++) {
            int n = rig.getSeedSampleCount(i);
            if (n < MIN_SAMPLES) {
                return false;
            }
            if (!usable(i) || n >= SwerveRig.SEED_SAMPLES) {
                continue;
            }
            double err = rig.getSeedStdErrorDegrees(i);
            double angle = rig.getSeedWheelDegrees(i);
            if (isRoughReading()) {
                if (err > Math.max(ROUGH_MIN_ERROR_DEG, ROUGH_FRACTION * Math.abs(angle))) {
                    return false;
                }
                continue;
            }
            if (err > PRECISE_ERROR_DEG) {
                allPrecise = false;
            }
            if (err > RECHECK_KNOWN_DEG) {
                allKnown = false;
            } else if (Math.abs(angle) - err > RECHECK_ABOVE_DEG) {
                farOff = true;
            }
        }
        return allPrecise
                || totalTime >= MEASURE_DEADLINE_S
                || (allKnown && farOff);
    }

    private void useReadings() {
        boolean rough = isRoughReading();
        readings++;
        boolean recheck = false;
        boolean allThere = true;
        for (int i = 0; i < 4; i++) {
            if (!usable(i)) {
                continue;
            }
            double angle = rig.getSeedWheelDegrees(i);
            double err = rig.getSeedStdErrorDegrees(i);

            // After a turn of more than a few degrees the wheel should have
            // moved toward 0. Moving away means the steering direction settings
            // disagree with the Through Bore.
            if (measured[i] && Math.abs(lastReadingDeg[i]) > 20.0) {
                double moved = wrap(angle - lastReadingDeg[i]);
                wrongWay[i] = Math.abs(moved) > 10.0 && Math.signum(moved) == Math.signum(lastReadingDeg[i]);
            }
            lastReadingDeg[i] = angle;

            // Combine with what's already known, each weighted by how exact it
            // is. The steering encoder then starts from the combined angle.
            double estimate;
            if (!measured[i] || wrongWay[i] || Double.isInfinite(knownDeg[i])) {
                estimate = angle;
                knownDeg[i] = err;
            } else {
                double before = rig.getHomeErrorDegrees(i);
                double wBefore = 1.0 / (knownDeg[i] * knownDeg[i]);
                double wNow = 1.0 / (err * err);
                estimate = before + wrap(angle - before) * wNow / (wBefore + wNow);
                knownDeg[i] = 1.0 / Math.sqrt(wBefore + wNow);
            }
            measured[i] = true;
            if (!rough) {
                // The rough reading seeds all four at once below.
                rig.nudgeWheel(i, wrap(estimate - rig.getHomeErrorDegrees(i)));
            }
            errorDeg[i] = -estimate;
            if (Math.abs(estimate) > RECHECK_ABOVE_DEG) {
                recheck = true;
            }
            if (Math.abs(estimate) > TURN_DONE_DEG) {
                allThere = false;
            }
        }

        if (rough) {
            // Steering encoders start from this reading, for the turns and for
            // driving after PLAY.
            rig.seedFromThroughBores();
            startTurn(false);
        } else if (recheck) {
            if (readings >= MAX_READINGS || totalTime >= MEASURE_DEADLINE_S) {
                finish("STOPPED: a wheel is still more than " + RECHECK_ABOVE_DEG + "° off");
            } else {
                startTurn(false);
            }
        } else if (allThere) {
            finish("all wheels forward");
        } else {
            startTurn(true);
        }
    }

    /** The turn has ended: measure again, or stop if there's no time for that. */
    private void turnEnded() {
        allPowerOff();
        for (int i = 0; i < 4; i++) {
            knownDeg[i] = Math.sqrt(knownDeg[i] * knownDeg[i] + TURN_ERROR_DEG * TURN_ERROR_DEG);
        }
        boolean timeForCheck = totalTime + SETTLE_S + CHECK_MIN_S <= MEASURE_DEADLINE_S;
        if (totalTime > TIME_LIMIT_S) {
            finish("time limit reached while turning");
        } else if (correction && (!timeForCheck || readings >= MAX_READINGS)) {
            finish("all wheels forward");
        } else {
            enter(State.SETTLE);
        }
    }

    private void startTurn(boolean smallCorrection) {
        correction = smallCorrection;
        holdTime = 0.0;
        for (int i = 0; i < 4; i++) {
            kick[i] = KICK_START;
            lastTogo[i] = -rig.getHomeErrorDegrees(i);
        }
        enter(State.TURN);
    }

    private void finish(String text) {
        StringBuilder noSignal = new StringBuilder();
        double worst = 0.0;
        for (int i = 0; i < 4; i++) {
            if (!usable(i)) {
                noSignal.append(' ').append(SwerveRig.LABELS[i]);
            } else {
                worst = Math.max(worst, knownDeg[i]);
            }
        }
        text = String.format("%s (%d readings, %.1f s, each wheel to about ±%.1f°)",
                text, readings, totalTime, worst);
        result = noSignal.length() == 0 ? text : text + ", but NO SIGNAL from:" + noSignal;
        allPowerOff();
        enter(State.DONE);
    }

    private void allPowerOff() {
        for (int i = 0; i < 4; i++) {
            rig.setSteerPowerDirect(i, 0.0);
        }
    }

    private void enter(State next) {
        state = next;
        stateTime = 0.0;
    }

    private static double wrap(double deg) {
        return Math.toDegrees(SwerveHardware.normalize(Math.toRadians(deg)));
    }

    public void addTelemetry(Telemetry telemetry) {
        switch (state) {
            case MEASURE:
                telemetry.addLine("FORWARD reading " + (readings + 1)
                        + (isRoughReading() ? " (rough)" : " (precise)") + ": measuring Through Bores, keep still");
                break;
            case TURN:
                telemetry.addLine(correction ? "FORWARD: small correction" : "FORWARD: turning to forward");
                break;
            case SETTLE:
                telemetry.addLine("FORWARD: letting the wheels stop");
                break;
            case DONE:
                telemetry.addLine("FORWARD DONE: " + result);
                break;
        }
        if (state != State.DONE) {
            telemetry.addLine(String.format("time so far: %.1f s (limit %.0f)   Through Bore samples/s: %.0f",
                    totalTime, TIME_LIMIT_S, samplesPerSecond));
        }
        telemetry.addLine("wheel | Through Bore | ± | samples | steer enc | status");
        for (int i = 0; i < 4; i++) {
            String status;
            if (!usable(i)) {
                status = "NO SIGNAL from " + SwerveRig.ENCODER_NAMES[i];
            } else if (wrongWay[i]) {
                status = "TURNED THE WRONG WAY: flip STEER_INVERTED[" + i + "] (all four: flip ENCODER_DIRECTION)";
            } else if (measured[i]) {
                status = String.format("was %.1f° off (±%.1f) at the last reading", -errorDeg[i], knownDeg[i]);
            } else {
                status = "";
            }
            telemetry.addData(SwerveRig.LABELS[i], "%6.1f° | ±%4.1f | %d | %6.1f° | %s",
                    rig.getSeedWheelDegrees(i), rig.getSeedStdErrorDegrees(i),
                    rig.getSeedSampleCount(i), rig.getHomeErrorDegrees(i), status);
        }
    }
}
