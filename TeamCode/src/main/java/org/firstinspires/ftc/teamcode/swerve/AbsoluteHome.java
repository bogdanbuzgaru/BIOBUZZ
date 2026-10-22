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
 * A Through Bore reading gets more accurate the longer it averages, so only
 * one reading is made accurate:
 *   1. ROUGH reading   quick: a wheel far from forward needs only a rough
 *                      angle to get close
 *   2. TURN            every wheel to forward on its steering encoder
 *   3. PRECISE reading to PRECISE_MAX_ERROR_DEG, now that the wheels are close
 *   4. TURN            the small correction that reading shows, then done
 * If the precise reading finds a wheel more than RECHECK_ABOVE_DEG off (it
 * didn't turn properly), steps 3 and 4 repeat.
 *
 * Tuning:
 *   - Too slow: raise PRECISE_MAX_ERROR_DEG (time goes with 1 / its square;
 *     3.0 -> 3.5 is about 25% faster, a little less exact). Telemetry shows
 *     samples/s; the more, the faster.
 *   - A wheel creeps to a stop short of 0° on the ground: raise MIN_POWER.
 *   - A wheel overshoots and swings back: lower POWER_PER_DEG.
 */
public class AbsoluteHome {

    /** One standard error of the precise reading, degrees: how exact FORWARD ends up. */
    private static final double PRECISE_MAX_ERROR_DEG = 3.0;
    /** The rough reading may be off by this fraction of the distance to forward... */
    private static final double ROUGH_FRACTION = 0.2;
    /** ...but needn't be better than this. */
    private static final double ROUGH_MIN_ERROR_DEG = 5.0;
    /** After a precise reading, check again only if a wheel was this far off. */
    private static final double RECHECK_ABOVE_DEG = 15.0;
    /** While measuring, sample the Through Bores this long per loop, ms. */
    private static final double SAMPLE_BURST_MS = 25.0;
    /** Fewest samples in a reading. */
    private static final int MIN_SAMPLES = 300;
    /** Time for the wheels to stop after turning before measuring. */
    private static final double SETTLE_S = 0.3;

    /** Most steering power while turning; enough for the wheels on the ground. */
    private static final double MAX_POWER = 0.8;
    /** Least steering power while still off target, to beat ground friction. */
    private static final double MIN_POWER = 0.3;
    /** Power per degree still to go; MAX_POWER from 30° out. */
    private static final double POWER_PER_DEG = MAX_POWER / 30.0;
    /** The turn is done when every wheel is this close by its steering encoder. */
    private static final double TURN_DONE_DEG = 1.5;
    /** Stop turning after this long even if a wheel isn't there. */
    private static final double TURN_TIMEOUT_S = 3.0;
    /** Give up after this many readings. */
    private static final int MAX_READINGS = 4;

    private enum State { MEASURE, TURN, SETTLE, DONE }

    private final SwerveRig rig;
    private State state = State.MEASURE;
    private double stateTime = 0.0;
    private double totalTime = 0.0;
    private int readings = 0;
    /** The turn under way is the last one: done when it ends. */
    private boolean finalTurn = false;
    private String result = "";

    private int samplesCounted = 0;
    private double rateClock = 0.0;
    private double samplesPerSecond = 0.0;

    private final double[] errorDeg = new double[4];
    private final boolean[] measured = new boolean[4];
    /** Through Bore angle at the last reading, to see which way the wheel really went. */
    private final double[] lastReadingDeg = new double[4];
    private final boolean[] wrongWay = new boolean[4];

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
                    // By the steering encoder, seeded from the last reading.
                    double togo = -rig.getHomeErrorDegrees(i);
                    if (Math.abs(togo) <= TURN_DONE_DEG) {
                        rig.setSteerPowerDirect(i, 0.0);
                        continue;
                    }
                    allThere = false;
                    double p = PIDController.clamp(POWER_PER_DEG * Math.abs(togo), MIN_POWER, MAX_POWER);
                    rig.setSteerPowerDirect(i, Math.copySign(p, togo));
                }
                if (allThere || stateTime > TURN_TIMEOUT_S) {
                    allPowerOff();
                    if (finalTurn) {
                        finish(String.format("all wheels forward (%d readings, %.1f s)", readings, totalTime));
                    } else {
                        enter(State.SETTLE);
                    }
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

    /** Every usable wheel's reading is accurate enough for this step. */
    private boolean readingsReady() {
        for (int i = 0; i < 4; i++) {
            int n = rig.getSeedSampleCount(i);
            if (n < MIN_SAMPLES) {
                return false;
            }
            if (!usable(i) || n >= SwerveRig.SEED_SAMPLES) {
                continue;
            }
            double needed = isRoughReading()
                    ? Math.max(ROUGH_MIN_ERROR_DEG, ROUGH_FRACTION * Math.abs(rig.getSeedWheelDegrees(i)))
                    : PRECISE_MAX_ERROR_DEG;
            if (rig.getSeedStdErrorDegrees(i) > needed) {
                return false;
            }
        }
        return true;
    }

    private void useReadings() {
        boolean rough = isRoughReading();
        readings++;
        boolean recheck = false;
        for (int i = 0; i < 4; i++) {
            if (!usable(i)) {
                continue;
            }
            double angle = rig.getSeedWheelDegrees(i);

            // After a turn of more than a few degrees the wheel should have
            // moved toward 0. Moving away means the steering direction settings
            // disagree with the Through Bore.
            if (measured[i] && Math.abs(lastReadingDeg[i]) > 20.0) {
                double moved = wrap(angle - lastReadingDeg[i]);
                wrongWay[i] = Math.abs(moved) > 10.0 && Math.signum(moved) == Math.signum(lastReadingDeg[i]);
            }
            measured[i] = true;
            lastReadingDeg[i] = angle;
            errorDeg[i] = -angle;
            if (Math.abs(errorDeg[i]) > RECHECK_ABOVE_DEG) {
                recheck = true;
            }
        }

        // Steering encoders start from this reading, for the turn and for
        // driving after PLAY.
        rig.seedFromThroughBores();

        if (rough) {
            enter(State.TURN);
        } else if (!recheck) {
            // Close everywhere: the steering encoders can be trusted for the
            // last small correction.
            finalTurn = true;
            enter(State.TURN);
        } else if (readings >= MAX_READINGS) {
            finish("STOPPED after " + MAX_READINGS + " readings, a wheel is still more than "
                    + RECHECK_ABOVE_DEG + "° off");
        } else {
            enter(State.TURN);
        }
    }

    private void finish(String text) {
        StringBuilder noSignal = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            if (!usable(i)) {
                noSignal.append(' ').append(SwerveRig.LABELS[i]);
            }
        }
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
                telemetry.addLine(finalTurn ? "FORWARD: last correction" : "FORWARD: turning to forward");
                break;
            case SETTLE:
                telemetry.addLine("FORWARD: letting the wheels stop");
                break;
            case DONE:
                telemetry.addLine("FORWARD DONE: " + result);
                break;
        }
        if (state != State.DONE) {
            telemetry.addLine(String.format("time so far: %.1f s   Through Bore samples/s: %.0f",
                    totalTime, samplesPerSecond));
        }
        telemetry.addLine("wheel | Through Bore | ± | samples | steer enc | status");
        for (int i = 0; i < 4; i++) {
            String status;
            if (!usable(i)) {
                status = "NO SIGNAL from " + SwerveRig.ENCODER_NAMES[i];
            } else if (wrongWay[i]) {
                status = "TURNED THE WRONG WAY: flip STEER_INVERTED[" + i + "] (all four: flip ENCODER_DIRECTION)";
            } else if (measured[i]) {
                status = String.format("was %.0f° off at the last reading", -errorDeg[i]);
            } else {
                status = "";
            }
            telemetry.addData(SwerveRig.LABELS[i], "%6.1f° | ±%4.1f | %d | %6.1f° | %s",
                    rig.getSeedWheelDegrees(i), rig.getSeedStdErrorDegrees(i),
                    rig.getSeedSampleCount(i), rig.getHomeErrorDegrees(i), status);
        }
    }
}
