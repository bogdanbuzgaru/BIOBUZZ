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
 * Each wheel goes on its own:
 *   1. READ    average its Through Bore while it stands still
 *   2. done    if it is within DONE_DEG of 0, it is left alone from then on
 *   3. TURN    otherwise turn it by exactly what that reading says is missing
 *   4. SETTLE  let it stop, then back to 1
 * A wheel keeps getting adjustments until it is home or time runs out.
 * A wheel far off gets a quick rough reading, a wheel close to 0 a precise one.
 *
 * Every turn ends with a + approach (see SteerApproach), so the Through
 * Bores' play is on the same side every time they're read after a turn.
 *
 * Gives up after TIME_LIMIT_S; telemetry then says which wheel isn't home.
 *
 * Tuning:
 *   - A wheel's correction barely changes its reading: raise SteerApproach.BACKLASH_TAKEUP_DEG.
 *   - A wheel overshoots its target a lot: lower MAX_POWER.
 */
public class AbsoluteHome {

    // ---------------- time ----------------

    /** Homing never takes longer than this, seconds. */
    private static final double TIME_LIMIT_S = 30.0;
    /** No new turn is started with less time than this left, seconds. */
    private static final double LAST_TURN_S = 3.0;

    // ---------------- readings ----------------

    /** A wheel this close to 0 by its Through Bore is home and isn't touched again, degrees. */
    private static final double DONE_DEG = 1.0;
    /**
     * A reading's own ± is allowed on top of DONE_DEG, up to this much, so a
     * wheel isn't turned over what may just be noise, degrees.
     */
    private static final double NOISE_ALLOWANCE_MAX_DEG = 1.0;
    /** A reading is good enough once known to this (one standard error), degrees... */
    private static final double PRECISE_ERROR_DEG = 0.5;
    /** ...or to this fraction of how far the wheel is from 0, if that's looser. */
    private static final double ROUGH_FRACTION = 0.2;
    /** A reading stops after this long however good it is, seconds. */
    private static final double READING_MAX_S = 5.0;
    /** While a wheel is reading and none is turning, sample the Through Bores this long per loop, ms. */
    private static final double SAMPLE_BURST_MS = 25.0;
    /** Fewest samples in a reading. */
    private static final int MIN_SAMPLES = 300;
    /** Time for a wheel to stop after turning before it is read. */
    private static final double SETTLE_S = 0.3;

    // ---------------- turning ----------------

    /** Most steering power while turning. */
    private static final double MAX_POWER = 1.0;
    /**
     * A turn toward 0 that leaves the wheel off on the same side by more than
     * this extra went the wrong way, degrees. Only shown on telemetry; the
     * wheel is still corrected. Large, so the Through Bore's play alone never
     * trips it.
     */
    private static final double WRONG_WAY_MARGIN_DEG = 10.0;

    private enum Step { READ, TURN, SETTLE, HOME, NO_SIGNAL }

    private final SwerveRig rig;
    private final SteerApproach approach;
    private final Step[] step = new Step[4];
    private final double[] stepTime = new double[4];
    private double totalTime = 0.0;
    private boolean finished = false;
    private String result = "";

    private int samplesCounted = 0;
    private double rateClock = 0.0;
    private double samplesPerSecond = 0.0;

    /** Readings taken of each wheel. */
    private final int[] readings = new int[4];
    /** Wheel angle at its last reading, degrees, and that reading's ±. */
    private final double[] readingDeg = new double[4];
    private final double[] readingErrorDeg = new double[4];
    private final boolean[] wrongWay = new boolean[4];
    /** Ran out of time before getting within DONE_DEG. */
    private final boolean[] outOfTime = new boolean[4];

    public AbsoluteHome(SwerveRig rig) {
        this.rig = rig;
        this.approach = new SteerApproach(rig, MAX_POWER);
        rig.restartSeedAverage();
        for (int i = 0; i < 4; i++) {
            step[i] = Step.READ;
        }
    }

    public boolean isDone() {
        return finished;
    }

    /** True while a steering motor is turning a wheel. */
    public boolean isTurning() {
        if (finished) {
            return false;
        }
        for (Step s : step) {
            if (s == Step.TURN) {
                return true;
            }
        }
        return false;
    }

    /** Stop homing where it is (e.g. PLAY was pressed first) and leave every steering encoder seeded. */
    public void stop() {
        if (!finished) {
            finish("stopped before every wheel was home");
        }
    }

    public void update(double dt) {
        if (finished) {
            allPowerOff();
            return;
        }
        totalTime += dt;
        approach.track();

        // Burst sampling holds up the loop, so only while no wheel is turning.
        boolean anyReading = false;
        for (Step s : step) {
            anyReading |= s == Step.READ;
        }
        countSamples(anyReading && !isTurning() ? rig.sampleEncodersFor(SAMPLE_BURST_MS) : 1, dt);

        boolean allHome = true;
        for (int i = 0; i < 4; i++) {
            stepTime[i] += dt;
            switch (step[i]) {
                case READ:
                    rig.setSteerPowerDirect(i, 0.0);
                    if (readingReady(i)) {
                        useReading(i);
                    }
                    break;
                case TURN:
                    approach.update(i, dt);
                    if (!approach.isTurning(i)) {
                        enter(i, Step.SETTLE);
                    }
                    break;
                case SETTLE:
                    rig.setSteerPowerDirect(i, 0.0);
                    if (stepTime[i] > SETTLE_S) {
                        rig.restartSeedAverage(i);
                        enter(i, Step.READ);
                    }
                    break;
                case HOME:
                case NO_SIGNAL:
                    rig.setSteerPowerDirect(i, 0.0);
                    break;
            }
            allHome &= step[i] == Step.HOME || step[i] == Step.NO_SIGNAL;
        }

        if (allHome) {
            finish("every wheel done");
        } else if (totalTime >= TIME_LIMIT_S) {
            finish("time limit reached");
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

    /** Is this wheel's reading good enough for how far it is from 0? */
    private boolean readingReady(int i) {
        int n = rig.getSeedSampleCount(i);
        if (n < MIN_SAMPLES) {
            return false;
        }
        if (!rig.isThroughBorePulsing(i) || n >= SwerveRig.SEED_SAMPLES || stepTime[i] >= READING_MAX_S) {
            return true;
        }
        double needed = Math.max(PRECISE_ERROR_DEG, ROUGH_FRACTION * Math.abs(rig.getSeedWheelDegrees(i)));
        return rig.getSeedStdErrorDegrees(i) <= needed;
    }

    /** Is the wheel home by its last reading, allowing for that reading's ±? */
    private boolean readsHome(int i) {
        return Math.abs(readingDeg[i]) <= DONE_DEG + Math.min(readingErrorDeg[i], NOISE_ALLOWANCE_MAX_DEG);
    }

    /** Turns this wheel has had: one per reading after the first. */
    private int turnsDone(int i) {
        return Math.max(0, readings[i] - 1);
    }

    private void useReading(int i) {
        if (!rig.isThroughBorePulsing(i)) {
            enter(i, Step.NO_SIGNAL);
            return;
        }
        double angle = rig.getSeedWheelDegrees(i);

        // After a turn the wheel should be near 0, or just past it. Further
        // off on the same side suggests the steering encoder and the Through
        // Bore disagree on which way is +. Shown on telemetry; the wheel is
        // still corrected.
        if (readings[i] > 0 && Math.signum(angle) == Math.signum(readingDeg[i])
                && Math.abs(angle) > Math.abs(readingDeg[i]) + WRONG_WAY_MARGIN_DEG) {
            wrongWay[i] = true;
        }
        readings[i]++;
        readingDeg[i] = angle;
        readingErrorDeg[i] = rig.getSeedStdErrorDegrees(i);

        // The steering encoder starts from this reading, for the turn and for
        // driving after PLAY.
        rig.seedFromThroughBore(i);
        approach.resetPosition(i);

        if (readsHome(i)) {
            enter(i, Step.HOME);
        } else if (TIME_LIMIT_S - totalTime < LAST_TURN_S) {
            outOfTime[i] = true;
            enter(i, Step.HOME);
        } else {
            approach.start(i, 0.0);
            enter(i, Step.TURN);
        }
    }

    private void finish(String text) {
        allPowerOff();
        StringBuilder notHome = new StringBuilder();
        StringBuilder noSignal = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            if (step[i] == Step.NO_SIGNAL) {
                noSignal.append(' ').append(SwerveRig.LABELS[i]);
                continue;
            }
            // A wheel never read at all: seed from what its average has.
            if (readings[i] == 0) {
                rig.seedFromThroughBore(i);
            }
            if (step[i] != Step.HOME || !readsHome(i)) {
                notHome.append(' ').append(SwerveRig.LABELS[i]);
            }
        }
        rig.finishSeeding();

        text = String.format("%s in %.1f s", text, totalTime);
        if (notHome.length() > 0) {
            text += ", NOT HOME:" + notHome;
        }
        if (noSignal.length() > 0) {
            text += ", NO SIGNAL from:" + noSignal;
        }
        result = text;
        finished = true;
    }

    private void allPowerOff() {
        for (int i = 0; i < 4; i++) {
            approach.cancel(i);
        }
    }

    private void enter(int i, Step next) {
        step[i] = next;
        stepTime[i] = 0.0;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (finished) {
            telemetry.addLine("FORWARD DONE: " + result);
        } else {
            telemetry.addLine(String.format("FORWARD: homing each wheel to within %.0f°   %.1f s (limit %.0f)   Through Bore samples/s: %.0f",
                    DONE_DEG, totalTime, TIME_LIMIT_S, samplesPerSecond));
        }
        telemetry.addLine("wheel | Through Bore | ± | samples | steer enc | status");
        for (int i = 0; i < 4; i++) {
            String status;
            switch (step[i]) {
                case READ:
                    status = String.format("reading %d (%.1f s), keep still", readings[i] + 1, stepTime[i]);
                    break;
                case TURN:
                    status = String.format("%s (%s)",
                            approach.isOnFinalLeg(i) ? "turning to 0" : "backing off past 0 (backlash)",
                            turnsDone(i) == 0 ? "first turn" : "adjustment " + turnsDone(i));
                    break;
                case SETTLE:
                    status = "letting it stop";
                    break;
                case HOME:
                    if (readsHome(i)) {
                        status = String.format("HOME: %.1f° (±%.1f) after %d readings", readingDeg[i], readingErrorDeg[i], readings[i]);
                    } else {
                        status = String.format("NOT HOME: out of time, %.1f° off (±%.1f)", readingDeg[i], readingErrorDeg[i]);
                    }
                    break;
                default:
                    status = "NO SIGNAL from " + SwerveRig.ENCODER_NAMES[i];
                    break;
            }
            if (finished && step[i] != Step.HOME && step[i] != Step.NO_SIGNAL) {
                status = String.format("NOT HOME: stopped, last read %.1f° off", readingDeg[i]);
            }
            if (wrongWay[i]) {
                status += "  (went the wrong way once: Through Bore and steering encoder may disagree on direction)";
            }
            telemetry.addData(SwerveRig.LABELS[i], "%6.1f° | ±%4.1f | %d | %6.1f° | %s",
                    rig.getSeedWheelDegrees(i), rig.getSeedStdErrorDegrees(i),
                    rig.getSeedSampleCount(i), rig.getHomeErrorDegrees(i), status);
        }
    }
}
