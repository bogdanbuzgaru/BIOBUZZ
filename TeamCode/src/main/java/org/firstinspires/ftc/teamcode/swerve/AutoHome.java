package org.firstinspires.ftc.teamcode.swerve;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turns every wheel to 0 degrees (straight forward) on its own during INIT.
 * Call update() once per INIT loop, after rig.sampleEncoders().
 *
 *   1. AVERAGING  wait until every Through Bore average is good to
 *                 AVERAGE_MAX_ERROR_DEG
 *   2. HOMING     seed the steering encoders from it and turn the wheels to 0
 *   3. SETTLING   let the wheels stop, then start a fresh average
 *   4. CHECKING   once that average is good, read where the wheels really
 *                 point. All within CHECK_TOLERANCE_DEG of 0: done. Otherwise
 *                 re-seed from this average and home again (up to MAX_PASSES).
 *
 * The wheels keep holding 0 from step 2 onwards.
 *
 * Tuning, if needed:
 *   - Wheels end up consistently off by the same angle: that wheel's offset
 *     (OpSwerveCalibration), not these numbers.
 *   - Takes too long: raise AVERAGE_MAX_ERROR_DEG (less accurate) or check the
 *     samples/s on telemetry; fewer than ~200/s makes every average slow.
 *   - Never finishes checking: raise CHECK_TOLERANCE_DEG a little. Below about
 *     2x AVERAGE_MAX_ERROR_DEG it can fail on noise alone.
 */
public class AutoHome {

    /** Each Through Bore average must be at least this good before it's used. */
    public static final double AVERAGE_MAX_ERROR_DEG = 3.0;
    /** The check passes when every wheel reads within this of 0. */
    public static final double CHECK_TOLERANCE_DEG = 6.0;
    /** Give up waiting for the wheels to reach 0 after this long. */
    public static final double HOMING_TIMEOUT_S = 4.0;
    /** Time for the wheels to stop moving before the check average starts. */
    public static final double SETTLE_S = 0.5;
    /** Home, check, re-home at most this many times. */
    public static final int MAX_PASSES = 3;

    private enum State { AVERAGING, HOMING, SETTLING, CHECKING, DONE }

    private final SwerveRig rig;
    private State state = State.AVERAGING;
    private double stateTime = 0.0;
    private int pass = 0;
    private String result = "";

    public AutoHome(SwerveRig rig) {
        this.rig = rig;
    }

    /** Start over from a fresh average. The wheels keep holding where they are. */
    public void restart() {
        rig.restartSeedAverage();
        state = State.AVERAGING;
        stateTime = 0.0;
        pass = 0;
        result = "";
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public void update(double dt) {
        stateTime += dt;
        // Holds 0 once homing has started; does nothing before that.
        boolean atHome = rig.updateHoming(dt);

        switch (state) {
            case AVERAGING:
                if (averagesReady()) {
                    startPass();
                }
                break;

            case HOMING:
                if (atHome || stateTime > HOMING_TIMEOUT_S) {
                    enter(State.SETTLING);
                }
                break;

            case SETTLING:
                if (stateTime > SETTLE_S) {
                    rig.restartSeedAverage();
                    enter(State.CHECKING);
                }
                break;

            case CHECKING:
                if (!averagesReady()) {
                    break;
                }
                double worst = worstWheelDegrees();
                if (Math.abs(worst) <= CHECK_TOLERANCE_DEG) {
                    result = String.format("all wheels forward (worst %.1f°, %d pass%s)",
                            worst, pass, pass == 1 ? "" : "es");
                    enter(State.DONE);
                } else if (pass < MAX_PASSES) {
                    // Re-seed from this fresh average and correct.
                    startPass();
                } else {
                    result = String.format("STOPPED after %d passes, a wheel still reads %.1f°. "
                            + "Check that wheel's offset and its steering motor.", pass, worst);
                    enter(State.DONE);
                }
                break;

            case DONE:
                break;
        }
    }

    private void startPass() {
        pass++;
        if (rig.startHoming()) {
            enter(State.HOMING);
        } else {
            // Shouldn't happen: averagesReady() checked the sample counts.
            restart();
        }
    }

    private void enter(State next) {
        state = next;
        stateTime = 0.0;
    }

    private boolean averagesReady() {
        for (int i = 0; i < 4; i++) {
            if (rig.getSeedSampleCount(i) < SwerveRig.MIN_SEED_SAMPLES
                    || rig.getSeedStdErrorDegrees(i) > AVERAGE_MAX_ERROR_DEG) {
                return false;
            }
        }
        return true;
    }

    /** The wheel angle furthest from 0, signed. */
    private double worstWheelDegrees() {
        double worst = 0.0;
        for (int i = 0; i < 4; i++) {
            double w = rig.getSeedWheelDegrees(i);
            if (Math.abs(w) > Math.abs(worst)) {
                worst = w;
            }
        }
        return worst;
    }

    public void addTelemetry(Telemetry telemetry) {
        switch (state) {
            case AVERAGING:
            case CHECKING:
                telemetry.addLine(String.format("AUTO-HOME %s: averaging until ±%.0f°   FL ±%.1f  FR ±%.1f  BL ±%.1f  BR ±%.1f",
                        state == State.AVERAGING ? "step 1" : "check, pass " + pass,
                        AVERAGE_MAX_ERROR_DEG,
                        rig.getSeedStdErrorDegrees(0), rig.getSeedStdErrorDegrees(1),
                        rig.getSeedStdErrorDegrees(2), rig.getSeedStdErrorDegrees(3)));
                break;
            case HOMING:
                telemetry.addLine(String.format("AUTO-HOME pass %d: turning to 0°   FL %.1f°  FR %.1f°  BL %.1f°  BR %.1f°",
                        pass, rig.getHomeErrorDegrees(0), rig.getHomeErrorDegrees(1),
                        rig.getHomeErrorDegrees(2), rig.getHomeErrorDegrees(3)));
                break;
            case SETTLING:
                telemetry.addLine("AUTO-HOME pass " + pass + ": letting the wheels settle");
                break;
            case DONE:
                telemetry.addLine("AUTO-HOME DONE: " + result);
                break;
        }
    }
}
