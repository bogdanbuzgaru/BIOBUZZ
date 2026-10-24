package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Timed moves without drive encoders. A drive move is: point the wheels the
 * way of travel, ramp the drive power up over RAMP_S, hold CRUISE_POWER for
 * the cruise time, ramp down over RAMP_S, then coast to a stop with the
 * wheels still pointing the way of travel. The same move always goes about
 * the same distance; the measured distances are in the calibration table.
 *
 * Heading is held by the Control Hub IMU at the heading given to
 * startDrive(), and the direction of travel is relative to heading 0 (the
 * last rig.resetHeading()), so several moves in a row keep one straight
 * frame instead of adding up small errors.
 *
 * Power is scaled by NOMINAL_VOLTAGE / battery voltage (the average of every
 * hub's battery), so a full and a tired battery go about the same distance.
 *
 * Directions: degrees from forward, counter-clockwise positive, so 0 is
 * forward, +90 left, -90 right, -45 diagonal front-right.
 *
 * Call update() once per loop, after rig.sampleEncoders().
 */
public class TimedMover {

    // ---------------- moves ----------------

    /** Drive power while cruising, at NOMINAL_VOLTAGE. */
    public static final double CRUISE_POWER = 0.4;
    /** Time to ramp the power up from 0, and down to 0 again, seconds. */
    public static final double RAMP_S = 1.0;
    /** Power is scaled to what it would be at this battery voltage. */
    private static final double NOMINAL_VOLTAGE = 12.5;

    /** Before driving, the wheels get this long to point the way of travel... */
    private static final double ALIGN_TIMEOUT_S = 1.0;
    /** ...and count as pointing that way within this, degrees. */
    private static final double ALIGN_TOLERANCE_DEG = 3.0;
    /**
     * After a move the drive motors coast and the wheels stay pointed the way
     * of travel this long, so the robot rolls to a stop by itself.
     */
    private static final double SETTLE_S = 1.5;

    // ---------------- calibration, measured 2026-10-23 ----------------
    // Two runs per direction (3 s and 6 s cruise, averaged over several tries):
    //   speed = (d6 - d3) / 3 s,  ramp distance = d3 - speed x 3 s
    // Left is taken to be the same as right. In between, linear interpolation.

    /** Off forward, degrees (either side). */
    private static final double[] CAL_DIR_DEG = {0.0, 45.0, 90.0};
    /** Cruise speed, cm/s: forward, diagonal, strafe. */
    private static final double[] CAL_SPEED_CM_S = {39.5, 37.0, 41.6};
    /** Distance the two ramps add, cm. */
    private static final double[] CAL_RAMP_CM = {25.8, 35.4, 8.0};

    // ---------------- heading ----------------

    /** Turning speed per degree of heading error while driving, rad/s per degree. */
    private static final double HOLD_KP = 0.04;
    /** Most turning speed used to hold the heading while driving, rad/s. */
    private static final double HOLD_MAX_RAD_S = 0.4;

    /** Turns: rad/s per degree still to go... */
    private static final double TURN_KP = 0.03;
    /** ...never faster than this, rad/s... */
    private static final double TURN_MAX_RAD_S = 0.8;
    /** ...nor slower than this until there, so friction doesn't stop it short. */
    private static final double TURN_MIN_RAD_S = 0.12;
    /** A turn is done within this many degrees... */
    private static final double TURN_DONE_DEG = 1.0;
    /** ...held for this long. */
    private static final double TURN_HOLD_S = 0.25;
    /** Give up on a turn after this long. */
    private static final double TURN_TIMEOUT_S = 6.0;

    public enum Phase { IDLE, ALIGN, DRIVE, TURN, SETTLE }

    private final SwerveRig rig;
    private final SwerveDrive drive;
    private final HardwareMap hardwareMap;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private Phase phase = Phase.IDLE;
    private double phaseTime = 0.0;
    private boolean finished = false;

    private double dirDeg = 0.0;
    private double cruiseS = 0.0;
    private double holdHeadingDeg = 0.0;
    private double turnTargetDeg = 0.0;
    private double turnHold = 0.0;
    private double drivePower = 0.0;

    private double voltage = NOMINAL_VOLTAGE;
    private boolean voltageRead = false;
    private double voltageSum = 0.0;
    private int voltageCount = 0;
    private double moveTime = 0.0;

    public TimedMover(SwerveRig rig, HardwareMap hardwareMap) {
        this.rig = rig;
        this.drive = rig.getDrive();
        this.hardwareMap = hardwareMap;
    }

    // ------------------------------------------------------------------
    // calibration
    // ------------------------------------------------------------------

    /** Measured cruise speed in this direction, cm/s. */
    public static double cruiseSpeedCmPerS(double dirDeg) {
        return interpolate(CAL_SPEED_CM_S, dirDeg);
    }

    /** Measured distance the two ramps add in this direction, cm. */
    public static double rampDistanceCm(double dirDeg) {
        return interpolate(CAL_RAMP_CM, dirDeg);
    }

    /**
     * Cruise time that drives this far in this direction. A distance shorter
     * than the ramps alone gets no cruise and goes a little too far.
     */
    public static double cruiseSecondsFor(double distanceM, double dirDeg) {
        double cruiseCm = distanceM * 100.0 - rampDistanceCm(dirDeg);
        return Math.max(0.0, cruiseCm / cruiseSpeedCmPerS(dirDeg));
    }

    private static double interpolate(double[] values, double dirDeg) {
        // Off forward, 0..90; backward counts as forward mirrored.
        double off = Math.abs(wrapDeg(dirDeg));
        if (off > 90.0) {
            off = 180.0 - off;
        }
        for (int i = 1; i < CAL_DIR_DEG.length; i++) {
            if (off <= CAL_DIR_DEG[i]) {
                double f = (off - CAL_DIR_DEG[i - 1]) / (CAL_DIR_DEG[i] - CAL_DIR_DEG[i - 1]);
                return values[i - 1] + f * (values[i] - values[i - 1]);
            }
        }
        return values[values.length - 1];
    }

    // ------------------------------------------------------------------
    // moves
    // ------------------------------------------------------------------

    /**
     * Drive one move.
     *
     * @param dirDeg         direction of travel relative to heading 0, degrees
     * @param cruiseSeconds  time at CRUISE_POWER between the ramps
     * @param holdHeadingDeg heading to keep the whole way, degrees
     */
    public void startDrive(double dirDeg, double cruiseSeconds, double holdHeadingDeg) {
        begin();
        this.dirDeg = dirDeg;
        this.cruiseS = cruiseSeconds;
        this.holdHeadingDeg = holdHeadingDeg;
        enter(Phase.ALIGN);
    }

    /** Turn on the spot to this heading, degrees. */
    public void startTurn(double targetHeadingDeg) {
        begin();
        turnTargetDeg = targetHeadingDeg;
        turnHold = 0.0;
        enter(Phase.TURN);
    }

    /** Stop at once (the driver took over); drive motors back to braking. */
    public void stop() {
        finished = false;
        enter(Phase.IDLE);
    }

    public boolean isBusy() {
        return phase != Phase.IDLE;
    }

    /** True once after a move has ended by itself (not by stop()). */
    public boolean consumeFinished() {
        boolean f = finished;
        finished = false;
        return f;
    }

    private void begin() {
        rig.setDrivePowerLimit(SwerveRig.MAX_DRIVE_POWER);
        drive.reset();
        finished = false;
        moveTime = 0.0;
        voltageSum = 0.0;
        voltageCount = 0;
        drivePower = 0.0;
    }

    public void update(double dt) {
        updateVoltage();
        if (phase == Phase.IDLE) {
            return;
        }
        phaseTime += dt;
        moveTime += dt;
        voltageSum += voltage;
        voltageCount++;

        switch (phase) {
            case ALIGN:
                alignWheels(dt);
                break;
            case DRIVE:
                driveMove(dt);
                break;
            case TURN:
                turnMove(dt);
                break;
            case SETTLE:
                // Zero speed: every wheel keeps the angle it had, motors coast.
                drive.drive(0.0, 0.0, 0.0, false, dt);
                if (phaseTime >= SETTLE_S) {
                    finished = true;
                    enter(Phase.IDLE);
                }
                break;
            default:
                break;
        }
    }

    /** Point every wheel the way of travel before any drive power. */
    private void alignWheels(double dt) {
        double dirRad = Math.toRadians(dirDeg - rig.getHeadingDegrees());
        boolean aligned = true;
        for (SwerveModule m : drive.getModules()) {
            m.setAngleOnly(dirRad, dt);
            // A wheel pointing exactly backwards is fine: it drives in reverse.
            double off = Math.abs(wrapDeg(Math.toDegrees(m.getAngleRadians() - dirRad)));
            if (Math.min(off, 180.0 - off) > ALIGN_TOLERANCE_DEG) {
                aligned = false;
            }
        }
        if (aligned || phaseTime >= ALIGN_TIMEOUT_S) {
            enter(Phase.DRIVE);
        }
    }

    /** Ramp up, cruise, ramp down, holding the heading and the direction of travel. */
    private void driveMove(double dt) {
        double t = phaseTime;
        double total = RAMP_S + cruiseS + RAMP_S;
        double profile;
        if (t < RAMP_S) {
            profile = t / RAMP_S;
        } else if (t < RAMP_S + cruiseS) {
            profile = 1.0;
        } else {
            profile = Math.max(0.0, (total - t) / RAMP_S);
        }
        drivePower = CRUISE_POWER * profile * NOMINAL_VOLTAGE / voltage;

        double heading = rig.getHeadingDegrees();
        double speed = speedForPower(drivePower);
        // Direction of travel is fixed to heading 0; turn it into the robot's
        // frame as it is now.
        double dirRad = Math.toRadians(dirDeg - heading);
        // Heading correction shrinks with the speed and stops with it. Alone,
        // it would steer the wheels to the turn-on-the-spot pattern (45°)
        // while the robot is still rolling, and it wobbles.
        double omega = 0.0;
        if (speed > 0.0) {
            omega = profile * clamp(HOLD_KP * wrapDeg(holdHeadingDeg - heading), -HOLD_MAX_RAD_S, HOLD_MAX_RAD_S);
        }

        drive.drive(speed * Math.cos(dirRad), speed * Math.sin(dirRad), omega, false, dt);

        if (t >= total) {
            drivePower = 0.0;
            rig.setDriveCoast(true);
            enter(Phase.SETTLE);
        }
    }

    /** Turn on the spot to turnTargetDeg by the IMU. */
    private void turnMove(double dt) {
        double togo = wrapDeg(turnTargetDeg - rig.getHeadingDegrees());
        double omega = 0.0;
        if (Math.abs(togo) > TURN_DONE_DEG) {
            double rate = clamp(Math.abs(togo) * TURN_KP, TURN_MIN_RAD_S, TURN_MAX_RAD_S);
            omega = Math.copySign(rate, togo);
            turnHold = 0.0;
        } else {
            turnHold += dt;
        }
        drive.drive(0.0, 0.0, omega, false, dt);

        // On timeout the end heading shows how far it got.
        if (turnHold >= TURN_HOLD_S || phaseTime >= TURN_TIMEOUT_S) {
            rig.setDriveCoast(true);
            enter(Phase.SETTLE);
        }
    }

    /**
     * Chassis speed that makes the modules send this drive power. With no
     * speed feedback a module sends DRIVE_KV x speed + DRIVE_KS.
     */
    private static double speedForPower(double power) {
        if (power <= SwerveConstants.DRIVE_KS) {
            return 0.0;
        }
        return (power - SwerveConstants.DRIVE_KS) / SwerveConstants.DRIVE_KV;
    }

    private void enter(Phase next) {
        if (next == Phase.IDLE) {
            drive.stop();
            drivePower = 0.0;
            rig.setDriveCoast(false);
        }
        phase = next;
        phaseTime = 0.0;
    }

    // ------------------------------------------------------------------
    // state, for telemetry
    // ------------------------------------------------------------------

    public Phase getPhase() {
        return phase;
    }

    public double getPhaseTime() {
        return phaseTime;
    }

    /** Seconds since the move started, alignment and settling included. */
    public double getMoveTime() {
        return moveTime;
    }

    /** Ramps plus cruise of the drive move under way, seconds. */
    public double getDriveSeconds() {
        return RAMP_S + cruiseS + RAMP_S;
    }

    public double getCruiseSeconds() {
        return cruiseS;
    }

    public double getTurnTargetDegrees() {
        return turnTargetDeg;
    }

    public double getDrivePower() {
        return drivePower;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getAverageVoltage() {
        return voltageCount == 0 ? voltage : voltageSum / voltageCount;
    }

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------

    /** Battery voltage, smoothed, read 10 times a second (each read costs a hub call). */
    private void updateVoltage() {
        if (voltageTimer.milliseconds() < 100) {
            return;
        }
        voltageTimer.reset();
        // Both hubs have their own battery: use the average of the two.
        double sum = 0.0;
        int n = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 7.0) {
                sum += v;
                n++;
            }
        }
        if (n > 0) {
            // First reading straight in, then smoothed.
            voltage = voltageRead ? voltage + 0.3 * (sum / n - voltage) : sum / n;
            voltageRead = true;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d > 180.0) {
            d -= 360.0;
        } else if (d < -180.0) {
            d += 360.0;
        }
        return d;
    }
}
