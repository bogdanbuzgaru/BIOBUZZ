package org.firstinspires.ftc.teamcode.swerve;

/**
 * Every number here is a starting point. Measure the geometry ones, tune the
 * gain ones.
 *
 * Motor data is from the goBILDA 5203 spec sheets. If you have 5202 or 5204
 * variants the numbers are the same for the ratios listed.
 */
public final class SwerveConstants {

    private SwerveConstants() {
    }

    // ------------------------------------------------------------------
    // motors
    // ------------------------------------------------------------------

    /** Drive: goBILDA 1150 RPM (5.2:1). 145.1 counts per output shaft rev. */
    public static final double DRIVE_TICKS_PER_REV = 145.1;
    public static final double DRIVE_FREE_RPM = 1150.0;
    public static final double DRIVE_STALL_TORQUE_NM = 0.775;   // 7.9 kg.cm

    /** Steering: goBILDA 312 RPM (19.2:1). Its own encoder is unused here. */
    public static final double STEER_TICKS_PER_REV = 537.7;
    public static final double STEER_FREE_RPM = 312.0;

    /** REV Through Bore quadrature, read via the steering motor's encoder port. */
    public static final double AZIMUTH_TICKS_PER_REV =
            ThroughBoreSteeringEncoder.COUNTS_PER_REV;   // 8192

    /**
     * Encoder revolutions per 360 degrees of module rotation.
     * Confirmed 1.0: the Through Bore sits on the module's final output, so it
     * reads the wheel's heading directly. No conversion, and no steering
     * gearbox backlash in the measurement.
     */
    public static final double AZIMUTH_GEAR_RATIO = 1.0;

    /**
     * Steering motor output revolutions per 360 degrees of module rotation.
     * 48:60 then 22:52  ->  1.25 x 2.3636 = 2.9545:1
     *
     * Not used by the control loop, since the encoder is on the azimuth axis.
     * It is here because it sets the azimuth slew rate:
     *   312 RPM / 2.9545 = 105.6 RPM = about 630 deg/s.
     * That is quick. Combined with the 90-degree flip optimization in
     * SwerveModule, no commanded move should take more than ~0.15 s.
     */
    public static final double STEER_GEAR_RATIO = (60.0 / 48.0) * (52.0 / 22.0);

    public static final double AZIMUTH_FREE_SPEED_DEG_PER_SEC =
            (STEER_FREE_RPM / STEER_GEAR_RATIO) * 360.0 / 60.0;

    // ------------------------------------------------------------------
    // drivetrain geometry: MEASURE THESE
    // ------------------------------------------------------------------

    /**
     * Motor output shaft revolutions per wheel revolution.
     * 24:36 then 30:48 then 12:24  ->  1.5 x 1.6 x 2.0 = 4.8:1
     *
     * This lands in a good place for 50 kg. See the note at the bottom.
     */
    public static final double DRIVE_GEAR_RATIO = (36.0 / 24.0) * (48.0 / 30.0) * (24.0 / 12.0);

    /** Three spur stages, roughly 95% each. */
    public static final double DRIVE_GEARBOX_EFFICIENCY = 0.86;

    public static final double WHEEL_DIAMETER_M = 0.096;   // 96 mm goBILDA
    public static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;
    public static final double WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0;

    /** Half the wheelbase (front to back) and half the track (left to right). */
    public static final double HALF_LENGTH_M = 0.18;
    public static final double HALF_WIDTH_M = 0.18;

    /** Module positions, front-left, front-right, back-left, back-right. */
    public static final double[] MODULE_X = {
            +HALF_LENGTH_M, +HALF_LENGTH_M, -HALF_LENGTH_M, -HALF_LENGTH_M
    };
    public static final double[] MODULE_Y = {
            +HALF_WIDTH_M, -HALF_WIDTH_M, +HALF_WIDTH_M, -HALF_WIDTH_M
    };

    public static final double ROBOT_MASS_KG = 50.0;

    /**
     * Per-module angle, in degrees, from your alignment reference to straight
     * forward, in the order fl, fr, bl, br.
     *
     * All zeros means you align each wheel to forward by eye. If you instead
     * park the modules against a jig or a chassis rail, run SwerveCalibration,
     * press Y on each module, and paste the line it prints here.
     */
    public static final double[] AZIMUTH_OFFSET_DEG = {0.0, 0.0, 0.0, 0.0};

    public static double[] azimuthOffsetsRadians() {
        double[] r = new double[AZIMUTH_OFFSET_DEG.length];
        for (int i = 0; i < r.length; i++) {
            r[i] = Math.toRadians(AZIMUTH_OFFSET_DEG[i]);
        }
        return r;
    }

    // ------------------------------------------------------------------
    // derived speed limits
    // ------------------------------------------------------------------

    /** Theoretical free speed. Under load you will see noticeably less. */
    public static final double THEORETICAL_MAX_SPEED_MPS =
            (DRIVE_FREE_RPM / DRIVE_GEAR_RATIO / 60.0) * WHEEL_CIRCUMFERENCE_M;

    /** Use ~85% of theoretical so the drive PID always has headroom to correct. */
    public static final double MAX_SPEED_MPS = 0.85 * THEORETICAL_MAX_SPEED_MPS;

    /** omega_max ~= v_max / distance from centre to a module. */
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =
            MAX_SPEED_MPS / Math.hypot(HALF_LENGTH_M, HALF_WIDTH_M);

    /** The heavy-robot knob. Raise until the wheels break traction, then back off. */
    /** Just under the sustainable figure below. Raise once it drives cleanly. */
    public static final double MAX_ACCEL_MPS2 = 1.2;
    public static final double MAX_ANGULAR_ACCEL_RAD_PER_SEC2 = 3.0;

    // ------------------------------------------------------------------
    // gains: starting values only
    // ------------------------------------------------------------------

    // Azimuth. Error in radians, output in motor power, so kP = 1.2 means full
    // power at about 48 degrees of error.
    public static final double STEER_KP = 1.2;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.06;
    public static final double STEER_KS = 0.05;   // stiction breakaway
    public static final double STEER_TOLERANCE_RAD = Math.toRadians(1.5);

    // Drive velocity. Feedforward does the work, the PID only trims it.
    public static final double DRIVE_KS = 0.10;
    public static final double DRIVE_KV = 1.0 / MAX_SPEED_MPS;
    public static final double DRIVE_KA = 0.0;
    public static final double DRIVE_KP = 0.4;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    // Heading hold. Error in radians, output in rad/s.
    public static final double HEADING_KP = 3.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.15;

    // ------------------------------------------------------------------
    // sanity checks: print these at init and look at them
    // ------------------------------------------------------------------

    /** Stall torque available at one wheel, N.m. */
    public static double wheelStallTorqueNm() {
        return DRIVE_STALL_TORQUE_NM * DRIVE_GEAR_RATIO * DRIVE_GEARBOX_EFFICIENCY;
    }

    /**
     * Torque per wheel at which the tyres break loose, N.m.
     * If wheelStallTorqueNm() exceeds this, you are traction limited, which is
     * the healthy place to be. If it is far below, you are torque limited and
     * no amount of PID tuning will help: add reduction.
     */
    public static double wheelTractionLimitNm(double frictionCoefficient) {
        return frictionCoefficient * ROBOT_MASS_KG * 9.81 * WHEEL_RADIUS_M / 4.0;
    }

    /**
     * Rough sustainable acceleration, m/s^2. Uses 25% of stall torque as the
     * continuous figure, because running these motors near stall draws 9.2 A
     * each, sags the battery and cooks the windings.
     */
    public static double sustainableAccelMps2() {
        double continuousTorquePerWheel = 0.25 * wheelStallTorqueNm();
        double forcePerWheel = continuousTorquePerWheel / WHEEL_RADIUS_M;
        return (4.0 * forcePerWheel) / ROBOT_MASS_KG;
    }

    /*
     * WHERE 4.8:1 PUTS YOU, assuming a 96 mm wheel
     *
     *   wheel speed                 1150 / 4.8 = 240 RPM
     *   top speed                   1.20 m/s
     *   stall torque at the wheel   3.2 N.m   (0.775 x 4.8 x 0.86)
     *   traction limit, mu = 0.8    4.7 N.m
     *   sustainable acceleration    1.33 m/s^2
     *   0 to top speed              about 0.9 s
     *
     * Stall torque sits at roughly two thirds of the traction limit. That is a
     * good place to be: the motors give out before the tyres do, so you will
     * not shred wheels or lose odometry to slip, and you still have enough
     * torque to actually move 50 kg at a useful rate. Nothing to change here.
     *
     * The one number still unverified is the wheel diameter. Everything above
     * scales with it: a 104 mm wheel raises top speed about 8% and drops
     * available force by the same, while a 72 mm wheel does the reverse.
     */
}
