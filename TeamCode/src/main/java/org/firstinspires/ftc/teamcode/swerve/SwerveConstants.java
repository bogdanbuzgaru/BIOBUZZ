package org.firstinspires.ftc.teamcode.swerve;
/**
 * Every number here is a starting point. Measure the geometry ones, tune the
 * gain ones. The tick counts are from the goBILDA Yellow Jacket spec sheets and
 * are worth double-checking against the exact part numbers you bought, because
 * they differ between the 5202 and 5203 series revisions.
 */
public final class SwerveConstants {

    private SwerveConstants() {
    }

    // ------------------------------------------------------------------
    // encoders
    // ------------------------------------------------------------------

    /** goBILDA 435 RPM (13.7:1), counts per revolution of the output shaft. */
    public static final double DRIVE_TICKS_PER_REV = 384.5;

    /** goBILDA 312 RPM (19.2:1), counts per revolution of the output shaft. */
    public static final double STEER_TICKS_PER_REV = 537.7;

    // ------------------------------------------------------------------
    // geometry: MEASURE THESE ON YOUR ROBOT
    // ------------------------------------------------------------------

    /** Motor output shaft revs per wheel rev. 1.0 if the wheel is on the shaft. */
    public static final double DRIVE_GEAR_RATIO = 1.0;

    /**
     * Steering motor output revs per full 360 degree rotation of the module.
     * This is the module's ring gear reduction. Common values are 2.0 to 4.0.
     * Getting this wrong makes the azimuth PID look untunable.
     */
    public static final double AZIMUTH_GEAR_RATIO = 2.0;

    public static final double WHEEL_DIAMETER_M = 0.096;  // 96 mm goBILDA
    public static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;

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

    // ------------------------------------------------------------------
    // speed limits
    // ------------------------------------------------------------------

    /**
     * Free speed. 435 RPM at the wheel with a 96 mm wheel is about 0.70 m/s of
     * theoretical top speed; under 50 kg of load expect meaningfully less, so
     * this is set below the theoretical figure on purpose. Measure yours by
     * driving a known distance and timing it.
     */
    public static final double MAX_SPEED_MPS = 0.60;

    /** omega_max ~= v_max / distance from centre to a module. */
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =
            MAX_SPEED_MPS / Math.hypot(HALF_LENGTH_M, HALF_WIDTH_M);

    /** The heavy-robot knob. Raise until the wheels break traction. */
    public static final double MAX_ACCEL_MPS2 = 1.5;
    public static final double MAX_ANGULAR_ACCEL_RAD_PER_SEC2 = 4.0;

    // ------------------------------------------------------------------
    // gains: starting values only
    // ------------------------------------------------------------------

    // Azimuth. Error is in radians, output is motor power, so kP of 1.0 means
    // full power at 1 radian (57 degrees) of error. That is roughly the right
    // order of magnitude to begin with.
    public static final double STEER_KP = 1.2;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.06;
    public static final double STEER_KS = 0.05;   // stiction breakaway
    public static final double STEER_TOLERANCE_RAD = Math.toRadians(2.0);

    // Drive velocity. Feedforward does the work.
    // kV = 1.0 / MAX_SPEED_MPS is the correct first guess: full power at top speed.
    public static final double DRIVE_KS = 0.08;
    public static final double DRIVE_KV = 1.0 / MAX_SPEED_MPS;
    public static final double DRIVE_KA = 0.0;
    public static final double DRIVE_KP = 0.4;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    // Heading hold. Error in radians, output in rad/s.
    public static final double HEADING_KP = 3.0;
    public static final double HEADING_KI = 0.0;
    public static final double HEADING_KD = 0.15;
}
