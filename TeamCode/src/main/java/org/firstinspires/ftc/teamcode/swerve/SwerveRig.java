package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.List;

/**
 * The swerve drive as wired on our robot: steering runs on each steering
 * motor's BUILT-IN encoder, and the REV Through Bore absolute encoders (white
 * wire on the digital ports) are only averaged during INIT to find where each
 * wheel points at the start.
 *
 * Used by OpSwerve and OpSwerveCalibration so both run the same code.
 *
 * Wheel angle from a Through Bore:
 *     wheel = ENCODER_DIRECTION x encoder + offset
 * The four offsets are saved on the robot (SwerveOffsets.txt) by
 * OpSwerveCalibration; until that file exists, DEFAULT_OFFSET_DEG is used.
 */
public class SwerveRig {

    // ---------------- robot configuration names, order: FL, FR, BL, BR ----------------

    public static final String[] DRIVE_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};
    public static final String[] STEER_NAMES = {"frontLeftsteer", "frontRightsteer", "backLeftsteer", "backRightsteer"};
    public static final String[] ENCODER_NAMES = {"EfrontLeft", "EfrontRight", "EbackLeft", "EbackRight"};
    public static final String[] LABELS = {"FL", "FR", "BL", "BR"};

    // ---------------- values to set on the robot ----------------

    /** Offsets used until OpSwerveCalibration has saved some, degrees. */
    public static final double[] DEFAULT_OFFSET_DEG = {0.0, 0.0, 0.0, 0.0};

    /** -1: when the encoder turns +15 degrees, the wheel turns -15. */
    public static final double ENCODER_DIRECTION = -1.0;

    /** True if a positive drive power moves that wheel BACKWARD. */
    public static final boolean[] DRIVE_INVERTED = {true, false, true, false};

    /** True if a positive steer power turns that module CLOCKWISE seen from above. */
    public static final boolean[] STEER_INVERTED = {false, false, false, false};

    /**
     * True if that steering motor's encoder count FALLS when the motor gets
     * positive power. If this is wrong the module spins or shakes; the safety
     * cut-off will say so on telemetry.
     */
    public static final boolean[] STEER_ENCODER_REVERSED = {false, false, false, false};

    // ---------------- steering safety cut-off ----------------

    /** Steering power above which the encoder must be seen moving the right way. */
    private static final double FAULT_POWER = 0.4;
    /** How long it may fail to before that module's steering is switched off. */
    private static final double FAULT_TIME_S = 1.0;
    /** Fewer counts than this during a fault means the encoder isn't counting at all. */
    private static final double FAULT_MIN_TICKS = 10.0;

    // ---------------- averaging ----------------

    /** Samples averaged in INIT to find the starting angle. */
    public static final int SEED_SAMPLES = 6000;
    /** Samples in the rolling average shown while running (about a second). */
    private static final int LIVE_SAMPLES = 300;
    /** Refuse to steer with fewer samples than this. */
    public static final int MIN_SEED_SAMPLES = 200;

    private static final String OFFSETS_FILE = "SwerveOffsets.txt";

    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final DcMotorEx[] steerMotors = new DcMotorEx[4];
    private final DigitalChannel[] encoders = new DigitalChannel[4];
    private final SteerMotorAngleSensor[] steerAngles = new SteerMotorAngleSensor[4];
    private final PwmDutyEstimator[] seed = new PwmDutyEstimator[4];
    private final PwmDutyEstimator[] live = new PwmDutyEstimator[4];
    private final SwerveModule[] modules = new SwerveModule[4];
    private final List<LynxModule> hubs;
    private final SwerveDrive drive;

    /** A wheel within this many degrees of 0 counts as homed. */
    public static final double HOME_TOLERANCE_DEG = 2.0;

    private boolean seeded = false;
    private boolean homing = false;

    private final double[] offsetDeg = new double[4];
    private final boolean offsetsFromFile;
    private String loadStatus = "";
    private String saveStatus = "not saved yet this run";

    private final double[] steerPowerSent = new double[4];
    private final double[] drivePowerSent = new double[4];
    private final boolean[] steerFault = new boolean[4];
    private final String[] faultReason = new String[4];
    private final double[] faultTime = new double[4];
    private final double[] faultTicks = new double[4];
    private final double[] lastSteerTicks = new double[4];

    public SwerveRig(HardwareMap hardwareMap) {

        // One bulk read per hub per loop. The digital inputs come back in the
        // same bulk read, so every sample reads all four encoders at once.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (int i = 0; i < 4; i++) {
            driveMotors[i] = hardwareMap.get(DcMotorEx.class, DRIVE_NAMES[i]);
            steerMotors[i] = hardwareMap.get(DcMotorEx.class, STEER_NAMES[i]);
            encoders[i] = hardwareMap.get(DigitalChannel.class, ENCODER_NAMES[i]);
            encoders[i].setMode(DigitalChannel.Mode.INPUT);

            if (DRIVE_INVERTED[i]) {
                driveMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (STEER_INVERTED[i]) {
                steerMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }

            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            steerMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            steerMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            steerMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            final int k = i;
            steerAngles[i] = new SteerMotorAngleSensor(
                    () -> steerMotors[k].getCurrentPosition(),
                    SwerveConstants.STEER_TICKS_PER_REV,
                    SwerveConstants.STEER_GEAR_RATIO,
                    STEER_ENCODER_REVERSED[i]);

            seed[i] = new PwmDutyEstimator(SEED_SAMPLES);
            live[i] = new PwmDutyEstimator(LIVE_SAMPLES);

            SwerveHardware.VelocitySensor driveVel = new SwerveHardware.EncoderVelocitySensor(
                    () -> driveMotors[k].getVelocity(),
                    SwerveConstants.DRIVE_TICKS_PER_REV,
                    SwerveConstants.DRIVE_GEAR_RATIO,
                    SwerveConstants.WHEEL_CIRCUMFERENCE_M);

            PIDController steerPid = new PIDController(
                    SwerveConstants.STEER_KP, SwerveConstants.STEER_KI, SwerveConstants.STEER_KD)
                    .setTolerance(SwerveConstants.STEER_TOLERANCE_RAD)
                    .setIZone(Math.toRadians(15))
                    .setMaxIntegralContribution(0.15)
                    .setDerivativeFilter(0.5);

            PIDController drivePid = new PIDController(
                    SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD)
                    .setOutputRange(-0.4, 0.4)
                    .setIZone(0.3)
                    .setMaxIntegralContribution(0.1);

            modules[i] = new SwerveModule(LABELS[i],
                    power -> {
                        driveMotors[k].setPower(power);
                        drivePowerSent[k] = power;
                    },
                    power -> {
                        double applied = steerFault[k] ? 0.0 : power;
                        steerMotors[k].setPower(applied);
                        steerPowerSent[k] = applied;
                    },
                    steerAngles[i], driveVel, steerPid, drivePid)
                    .setDriveFeedforward(SwerveConstants.DRIVE_KS,
                            SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA)
                    .setSteerStaticFeedforward(SwerveConstants.STEER_KS)
                    .setCosineCompensation(true);
        }

        // No gyro: robot-centric driving, no heading hold.
        drive = new SwerveDrive(modules,
                SwerveConstants.MODULE_X, SwerveConstants.MODULE_Y, null,
                SwerveConstants.MAX_SPEED_MPS, SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC)
                .setMaxAcceleration(SwerveConstants.MAX_ACCEL_MPS2)
                .setMaxAngularAcceleration(SwerveConstants.MAX_ANGULAR_ACCEL_RAD_PER_SEC2);

        offsetsFromFile = loadOffsets();
    }

    // ------------------------------------------------------------------
    // Through Bore sampling and seeding
    // ------------------------------------------------------------------

    /** Fresh bulk read, then one sample of every Through Bore. Call once per loop. */
    public void sampleEncoders() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        for (int i = 0; i < 4; i++) {
            boolean high = encoders[i].getState();
            seed[i].addSample(high);
            live[i].addSample(high);
        }
    }

    /**
     * Sample the Through Bores over and over for about ms milliseconds. Only
     * the hub(s) with the encoders are read, so this is several times faster
     * than one sample per loop.
     *
     * @return how many samples were taken
     */
    public int sampleEncodersFor(double ms) {
        long end = System.nanoTime() + (long) (ms * 1e6);
        int n = 0;
        do {
            sampleEncoders();
            n++;
        } while (System.nanoTime() < end);
        return n;
    }

    /** Start the INIT average again, e.g. after the robot was bumped. */
    public void restartSeedAverage() {
        for (PwmDutyEstimator e : seed) {
            e.reset();
        }
    }

    /**
     * Tell every steering encoder where its wheel points, from the INIT average.
     *
     * @return false if any encoder has too few samples to trust
     */
    public boolean seedFromThroughBores() {
        boolean enough = true;
        for (int i = 0; i < 4; i++) {
            if (seed[i].getSampleCount() < MIN_SEED_SAMPLES) {
                enough = false;
            }
            steerAngles[i].setCurrentAngle(Math.toRadians(wheelDegreesFrom(seed[i], i)));
        }
        drive.reset();
        for (int i = 0; i < 4; i++) {
            lastSteerTicks[i] = steerAngles[i].getTicks();
        }
        seeded = enough;
        return enough;
    }

    /** Samples in a Through Bore's INIT average. */
    public int getSeedSampleCount(int i) {
        return seed[i].getSampleCount();
    }

    /** +- uncertainty of a Through Bore's INIT average, degrees. */
    public double getSeedStdErrorDegrees(int i) {
        return seed[i].getStdErrorDegrees();
    }

    /**
     * True if a Through Bore's INIT average saw its signal both high and low.
     * Always low or always high means no pulses are arriving (white wire
     * unplugged or on the wrong port), and its angle means nothing.
     */
    public boolean isThroughBorePulsing(int i) {
        double duty = seed[i].getDuty();
        return duty > 0.002 && duty < 0.998;
    }

    /** Wheel angle from a Through Bore's INIT average, after direction and offset, -180..180. */
    public double getSeedWheelDegrees(int i) {
        return wheelDegreesFrom(seed[i], i);
    }

    /**
     * Tell every steering encoder where its wheel points, from known angles
     * (e.g. read back by WheelAngleStore). Doesn't use the Through Bores.
     */
    public void seedFromAngles(double[] wheelDeg) {
        for (int i = 0; i < 4; i++) {
            steerAngles[i].setCurrentAngle(Math.toRadians(wheelDeg[i]));
        }
        drive.reset();
        for (int i = 0; i < 4; i++) {
            lastSteerTicks[i] = steerAngles[i].getTicks();
        }
        seeded = true;
    }

    /**
     * Start turning every wheel to exactly 0 degrees from wherever it's been
     * seeded. Keep calling updateHoming() each loop.
     *
     * @return false if the steering encoders haven't been seeded; nothing moves
     */
    public boolean startHomingToZero() {
        homing = seeded;
        return homing;
    }

    /**
     * Shift what one steering encoder believes by deltaDeg. While the wheel is
     * held at 0 (homing), the steering then turns it by -deltaDeg to match: a
     * way to move a wheel to where 0 really is, and have that become 0.
     */
    public void nudgeWheel(int i, double deltaDeg) {
        steerAngles[i].setCurrentAngle(steerAngles[i].getAngleRadians() + Math.toRadians(deltaDeg));
    }

    /** Every wheel's angle by its steering encoder, degrees, -180..180, order FL FR BL BR. */
    public double[] getWheelAnglesDegrees() {
        double[] deg = new double[4];
        for (int i = 0; i < 4; i++) {
            deg[i] = Math.toDegrees(steerAngles[i].getAngleRadians());
        }
        return deg;
    }

    /** True once the steering encoders have been seeded from enough samples. */
    public boolean isSeeded() {
        return seeded;
    }

    // ------------------------------------------------------------------
    // homing: turn every wheel to 0 degrees (straight forward)
    // ------------------------------------------------------------------

    /**
     * Seed from the Through Bore average collected so far, then start turning
     * every wheel to exactly 0 degrees. Keep calling updateHoming() each loop.
     * Calling it again re-seeds from a fresh average and homes again, which
     * corrects any error left by the first seed.
     *
     * @return false if there weren't enough samples yet; nothing moves
     */
    public boolean startHoming() {
        homing = seedFromThroughBores();
        return homing;
    }

    /**
     * Drive every wheel toward 0 degrees, never via the 180 degree shortcut,
     * so each one really ends up pointing forward.
     *
     * @return true when every wheel is within HOME_TOLERANCE_DEG of 0
     */
    public boolean updateHoming(double dt) {
        if (!homing) {
            return false;
        }
        for (SwerveModule m : modules) {
            m.setAngleOnly(0.0, dt, false);
        }
        return isAtHome();
    }

    public boolean isHoming() {
        return homing;
    }

    public boolean isAtHome() {
        for (int i = 0; i < 4; i++) {
            if (Math.abs(getHomeErrorDegrees(i)) > HOME_TOLERANCE_DEG) {
                return false;
            }
        }
        return true;
    }

    /** How far a wheel is from 0 degrees, by its steering encoder. */
    public double getHomeErrorDegrees(int i) {
        return Math.toDegrees(steerAngles[i].getAngleRadians());
    }

    /** Wheel angle in degrees, -180..180, from an encoder estimate. */
    private double wheelDegreesFrom(PwmDutyEstimator estimator, int i) {
        double deg = ENCODER_DIRECTION * estimator.getAngleDegrees() + offsetDeg[i];
        return Math.toDegrees(SwerveHardware.normalize(Math.toRadians(deg)));
    }

    // ------------------------------------------------------------------
    // offsets
    // ------------------------------------------------------------------

    public double getOffsetDegrees(int i) {
        return offsetDeg[i];
    }

    /** True once every Through Bore's INIT average holds the full SEED_SAMPLES. */
    public boolean seedAveragesFull() {
        for (PwmDutyEstimator e : seed) {
            if (e.getSampleCount() < SEED_SAMPLES) {
                return false;
            }
        }
        return true;
    }

    /**
     * Declare that every wheel is pointing straight forward right now: each
     * offset is set so the current Through Bore average reads as a 0° wheel
     * (offset = -ENCODER_DIRECTION x encoder), then saved.
     *
     * @return false if the save failed; getSaveStatus() says why
     */
    public boolean captureForwardOffsets() {
        for (int i = 0; i < 4; i++) {
            double deg = -ENCODER_DIRECTION * seed[i].getAngleDegrees();
            offsetDeg[i] = Math.toDegrees(SwerveHardware.normalize(Math.toRadians(deg)));
        }
        return saveOffsets();
    }

    public boolean offsetsCameFromFile() {
        return offsetsFromFile;
    }

    /** Where the offsets came from at startup, or why the saved file wasn't used. */
    public String getLoadStatus() {
        return loadStatus;
    }

    /** Result of the last save, checked by reading the file back. */
    public String getSaveStatus() {
        return saveStatus;
    }

    /** One line to paste into DEFAULT_OFFSET_DEG if you want them in the code too. */
    public String offsetsAsCode() {
        return String.format("{%.1f, %.1f, %.1f, %.1f}",
                offsetDeg[0], offsetDeg[1], offsetDeg[2], offsetDeg[3]);
    }

    private File offsetsFile() {
        return AppUtil.getInstance().getSettingsFile(OFFSETS_FILE);
    }

    private boolean loadOffsets() {
        System.arraycopy(DEFAULT_OFFSET_DEG, 0, offsetDeg, 0, 4);
        File file = offsetsFile();
        if (!file.exists()) {
            loadStatus = "defaults from code (nothing saved yet at " + file.getPath() + ")";
            return false;
        }
        try {
            double[] read = parseOffsets(ReadWriteFile.readFileOrThrow(file));
            System.arraycopy(read, 0, offsetDeg, 0, 4);
            loadStatus = "loaded from " + file.getPath();
            return true;
        } catch (Exception e) {
            loadStatus = "defaults from code (couldn't read " + file.getPath() + ": " + e.getMessage() + ")";
            return false;
        }
    }

    /**
     * Writes the offsets and reads them straight back, so a failed save shows
     * on telemetry instead of disappearing into the robot log.
     */
    public boolean saveOffsets() {
        File file = offsetsFile();
        try {
            ReadWriteFile.writeFileOrThrow(file, offsetDeg[0] + "\n" + offsetDeg[1] + "\n"
                    + offsetDeg[2] + "\n" + offsetDeg[3] + "\n");
            double[] back = parseOffsets(ReadWriteFile.readFileOrThrow(file));
            for (int i = 0; i < 4; i++) {
                if (Math.abs(back[i] - offsetDeg[i]) > 1e-9) {
                    saveStatus = "SAVE FAILED: file reads back different values";
                    return false;
                }
            }
            saveStatus = "saved to " + file.getPath();
            return true;
        } catch (Exception e) {
            saveStatus = "SAVE FAILED: " + e.getMessage();
            return false;
        }
    }

    private static double[] parseOffsets(String text) {
        String[] parts = text.trim().split("\\s+");
        if (parts.length != 4) {
            throw new IllegalArgumentException("expected 4 numbers, found " + parts.length);
        }
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = Double.parseDouble(parts[i]);
        }
        return values;
    }

    // ------------------------------------------------------------------
    // direct steering power (no steering encoder involved)
    // ------------------------------------------------------------------

    /**
     * Power one steering motor directly, drive motor off. Used by
     * AbsoluteHome. A module the steering safety cut-off switched off gets 0.
     *
     * Only sends what changed: every motor command takes the hub time that
     * would otherwise go to sampling the Through Bores.
     */
    public void setSteerPowerDirect(int i, double power) {
        if (drivePowerSent[i] != 0.0) {
            driveMotors[i].setPower(0.0);
            drivePowerSent[i] = 0.0;
        }
        if (steerFault[i]) {
            power = 0.0;
        }
        if (steerPowerSent[i] != power) {
            steerMotors[i].setPower(power);
            steerPowerSent[i] = power;
        }
    }

    // ------------------------------------------------------------------
    // motors off
    // ------------------------------------------------------------------

    /**
     * Zero power on all eight motors and let them coast, so the wheels can be
     * turned by hand. Nothing in this class powers them again unless the drive
     * or homing is used.
     */
    public void floatAllMotors() {
        for (int i = 0; i < 4; i++) {
            driveMotors[i].setPower(0.0);
            steerMotors[i].setPower(0.0);
            drivePowerSent[i] = 0.0;
            steerPowerSent[i] = 0.0;
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            steerMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    // ------------------------------------------------------------------
    // steering safety cut-off
    // ------------------------------------------------------------------

    /**
     * Switches a module's steering off when its encoder doesn't follow the
     * motor: pushed hard for FAULT_TIME_S while the count stands still (cable in
     * the wrong port, or unplugged) or runs the wrong way (reversed count).
     * Either one makes the module spin or shake. Call once per loop.
     */
    public void checkSteering(double dt) {
        for (int i = 0; i < 4; i++) {
            double ticks = steerAngles[i].getTicks();
            double moved = ticks - lastSteerTicks[i];
            lastSteerTicks[i] = ticks;
            if (steerFault[i]) {
                continue;
            }

            double power = steerPowerSent[i];
            if (Math.abs(power) < FAULT_POWER || moved * power > 0) {
                faultTime[i] = 0.0;
                faultTicks[i] = 0.0;
                continue;
            }

            faultTime[i] += dt;
            faultTicks[i] += Math.copySign(moved, power);   // negative = wrong way
            if (faultTime[i] > FAULT_TIME_S) {
                steerFault[i] = true;
                faultReason[i] = Math.abs(faultTicks[i]) < FAULT_MIN_TICKS
                        ? "encoder not counting. Check " + STEER_NAMES[i]
                          + "'s encoder cable is in ITS OWN motor port."
                        : "encoder counts backwards. Flip STEER_ENCODER_REVERSED[" + i + "].";
            }
        }
    }

    public void clearSteeringFaults() {
        for (int i = 0; i < 4; i++) {
            steerFault[i] = false;
            faultTime[i] = 0.0;
            faultTicks[i] = 0.0;
        }
    }

    // ------------------------------------------------------------------
    // telemetry
    // ------------------------------------------------------------------

    public void addFaultTelemetry(Telemetry telemetry) {
        for (int i = 0; i < 4; i++) {
            if (steerFault[i]) {
                telemetry.addLine("!! " + LABELS[i] + " STEERING OFF: " + faultReason[i]);
            }
        }
    }

    /**
     * REV Through Bore encoders. Each angle is an average, with its +-
     * uncertainty, and is only accurate while the wheel is still.
     *
     * In INIT (initAverage true) each line shows two positions:
     *   avg - everything since INIT (or since CROSS restarted it); this is what
     *         seeds the steering, and it stops being meaningful once the wheel
     *         moves, e.g. during homing
     *   now - the last ~1 s, so it follows the wheel to its new position
     * plus the wheel angle from avg (after direction and offset) and the
     * samples in avg. When running, only the ~1 s position is shown.
     */
    public void addThroughBoreTelemetry(Telemetry telemetry, boolean initAverage, double sampleRateHz) {
        if (initAverage) {
            telemetry.addLine(String.format(
                    "THROUGH BORE  (avg | now | wheel from avg | samples)   %.0f samples/s",
                    sampleRateHz));
            for (int i = 0; i < 4; i++) {
                telemetry.addData(ENCODER_NAMES[i], "%6.1f° ±%4.1f | %6.1f° ±%4.1f | %6.1f° | %d",
                        seed[i].getAngleDegrees(), seed[i].getStdErrorDegrees(),
                        live[i].getAngleDegrees(), live[i].getStdErrorDegrees(),
                        wheelDegreesFrom(seed[i], i), seed[i].getSampleCount());
            }
            return;
        }
        telemetry.addLine(String.format("THROUGH BORE  (encoder | wheel | samples)   %.0f samples/s",
                sampleRateHz));
        for (int i = 0; i < 4; i++) {
            telemetry.addData(ENCODER_NAMES[i], "%6.1f° ±%4.1f | %6.1f° | %d",
                    live[i].getAngleDegrees(), live[i].getStdErrorDegrees(),
                    wheelDegreesFrom(live[i], i), live[i].getSampleCount());
        }
    }

    /**
     * Steering motors' built-in encoders: port, raw count, counts per second, the
     * wheel angle the steering loop believes, the power being sent, and whether
     * the count moves the way that power should move it.
     */
    public void addSteerEncoderTelemetry(Telemetry telemetry) {
        telemetry.addLine("STEER MOTOR ENCODERS  (port | count | counts/s | wheel | power | check)");
        for (int i = 0; i < 4; i++) {
            // Checked with the reversal applied, as the steering loop sees it.
            double sign = STEER_ENCODER_REVERSED[i] ? -1.0 : 1.0;
            double velocity = sign * steerMotors[i].getVelocity();
            double power = steerPowerSent[i];
            telemetry.addData(STEER_NAMES[i], "p%d | %7d | %6.0f | %6.1f° | %5.2f | %s",
                    steerMotors[i].getPortNumber(),
                    steerMotors[i].getCurrentPosition(), velocity,
                    Math.toDegrees(steerAngles[i].getAngleRadians()),
                    power, directionCheck(power, velocity));
        }
    }

    /** Does the encoder move the way the motor is being pushed? */
    private static String directionCheck(double power, double countsPerSecond) {
        if (Math.abs(power) < 0.1) {
            return "idle";
        }
        if (Math.abs(countsPerSecond) < 20.0) {
            return "NOT COUNTING";
        }
        return countsPerSecond * power > 0 ? "ok" : "BACKWARDS";
    }

    // ------------------------------------------------------------------

    public SwerveDrive getDrive() {
        return drive;
    }

    public SwerveModule getModule(int i) {
        return modules[i];
    }
}
