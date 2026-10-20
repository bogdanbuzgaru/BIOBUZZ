package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.PIDController;
import org.firstinspires.ftc.teamcode.swerve.PwmDutyEstimator;
import org.firstinspires.ftc.teamcode.swerve.SteerMotorAngleSensor;
import org.firstinspires.ftc.teamcode.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveHardware;
import org.firstinspires.ftc.teamcode.swerve.SwerveModule;

import java.util.List;

/**
 * Swerve test TeleOp.
 *
 * Steering runs on each steering motor's BUILT-IN encoder. The REV Through Bore
 * absolute encoders (white wire on the digital ports) are only used during INIT,
 * to find where each wheel points before the match starts.
 *
 * HOW TO RUN
 *   1. Put the robot down and press INIT. Don't touch it: the absolute encoders
 *      are averaged while it sits still. Press A to restart the average if the
 *      robot got bumped.
 *   2. Wait until each wheel's +- figure is small (a few degrees), then PLAY.
 *   3. Left stick drives, right stick turns.
 *
 * SETTING THE OFFSETS (ENCODER_OFFSET_DEG below)
 *   Point a wheel straight forward, INIT, and read that wheel's "encoder"
 *   value. Put that number in its slot. The wheel angle is then
 *       wheel = ENCODER_DIRECTION x encoder + offset
 *   which reads 0 with the wheel forward. ENCODER_DIRECTION is -1 because the
 *   encoder turns opposite to the wheel.
 *
 * Driving is robot-centric (stick forward = robot's front).
 */
@TeleOp(name = "OpSwerve", group = "drive")
public class OpSwerve extends LinearOpMode {

    // ---------------- robot configuration names, order: FL, FR, BL, BR ----------------

    private static final String[] DRIVE_NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};
    private static final String[] STEER_NAMES = {"frontLeftsteer", "frontRightsteer", "backLeftsteer", "backRightsteer"};
    private static final String[] ENCODER_NAMES = {"EfrontLeft", "EfrontRight", "EbackLeft", "EbackRight"};
    private static final String[] LABELS = {"FL", "FR", "BL", "BR"};

    // ---------------- values to set on the robot ----------------

    /** Degrees added to each wheel's angle so it reads 0 with the wheel straight forward. */
    public static double[] ENCODER_OFFSET_DEG = {0.0, 0.0, 0.0, 0.0};

    /** -1: when the encoder turns +15 degrees, the wheel turns -15. */
    public static double ENCODER_DIRECTION = -1.0;

    /** True if a positive drive power moves that wheel BACKWARD. */
    public static boolean[] DRIVE_INVERTED = {false, false, false, false};

    /** True if a positive steer power turns that module CLOCKWISE seen from above. */
    public static boolean[] STEER_INVERTED = {false, false, false, false};

    /**
     * True if that steering motor's encoder count FALLS when the motor gets
     * positive power. If this is wrong the module spins or shakes; the safety
     * cut-off below will say so on telemetry.
     */
    public static boolean[] STEER_ENCODER_REVERSED = {false, false, false, false};

    // ---------------- steering safety cut-off ----------------

    /** Steering power above which the encoder must be seen moving the right way. */
    private static final double FAULT_POWER = 0.4;
    /** How long it may fail to before that module's steering is switched off. */
    private static final double FAULT_TIME_S = 1.0;
    /** Fewer counts than this during a fault means the encoder isn't counting at all. */
    private static final double FAULT_MIN_TICKS = 10.0;

    // ---------------- averaging ----------------

    /** Samples averaged in INIT to find the starting angle. */
    private static final int SEED_SAMPLES = 6000;
    /** Samples in the rolling average shown while driving (about a second). */
    private static final int LIVE_SAMPLES = 300;
    /** Refuse to drive with fewer samples than this. */
    private static final int MIN_SEED_SAMPLES = 200;

    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final DcMotorEx[] steerMotors = new DcMotorEx[4];
    private final DigitalChannel[] encoders = new DigitalChannel[4];
    private final SteerMotorAngleSensor[] steerAngles = new SteerMotorAngleSensor[4];
    private final PwmDutyEstimator[] seed = new PwmDutyEstimator[4];
    private final PwmDutyEstimator[] live = new PwmDutyEstimator[4];
    private final SwerveModule[] modules = new SwerveModule[4];
    private List<LynxModule> hubs;

    private final boolean[] steerFault = new boolean[4];
    private final String[] faultReason = new String[4];
    private final double[] faultTime = new double[4];
    private final double[] faultTicks = new double[4];
    private final double[] lastSteerTicks = new double[4];

    @Override
    public void runOpMode() {

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
                    power -> driveMotors[k].setPower(power),
                    power -> steerMotors[k].setPower(steerFault[k] ? 0.0 : power),
                    steerAngles[i], driveVel, steerPid, drivePid)
                    .setDriveFeedforward(SwerveConstants.DRIVE_KS,
                            SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA)
                    .setSteerStaticFeedforward(SwerveConstants.STEER_KS)
                    .setCosineCompensation(true);
        }

        // No gyro: robot-centric driving, no heading hold.
        SwerveDrive drive = new SwerveDrive(modules,
                SwerveConstants.MODULE_X, SwerveConstants.MODULE_Y, null,
                SwerveConstants.MAX_SPEED_MPS, SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC)
                .setMaxAcceleration(SwerveConstants.MAX_ACCEL_MPS2)
                .setMaxAngularAcceleration(SwerveConstants.MAX_ANGULAR_ACCEL_RAD_PER_SEC2);

        // ---------------- INIT: average the absolute encoders ----------------
        ElapsedTime telemetryTimer = new ElapsedTime();
        ElapsedTime rateTimer = new ElapsedTime();
        int samplesSinceRate = 0;
        double sampleRateHz = 0.0;

        while (opModeInInit()) {
            sampleEncoders();
            samplesSinceRate++;

            if (gamepad1.a) {
                for (PwmDutyEstimator e : seed) {
                    e.reset();
                }
            }

            // Telemetry is slow; only send it a few times a second so the
            // loop spends its time sampling.
            if (telemetryTimer.milliseconds() > 200) {
                sampleRateHz = samplesSinceRate / rateTimer.seconds();
                samplesSinceRate = 0;
                rateTimer.reset();
                telemetryTimer.reset();

                telemetry.addLine("Keep the robot STILL. A = restart average. PLAY when +- is small.");
                addThroughBoreTelemetry(seed, sampleRateHz);
                addSteerEncoderTelemetry();
                telemetry.update();
            }
        }

        if (isStopRequested()) return;

        // ---------------- START: seed the motor encoders ----------------
        boolean seeded = true;
        for (int i = 0; i < 4; i++) {
            if (seed[i].getSampleCount() < MIN_SEED_SAMPLES) {
                seeded = false;
            }
            steerAngles[i].setCurrentAngle(Math.toRadians(wheelDegreesFrom(seed[i], i)));
        }

        if (!seeded) {
            // Steering from a guess would send every wheel somewhere random.
            while (opModeIsActive()) {
                drive.stop();
                telemetry.addLine("NOT ENOUGH ENCODER SAMPLES - stay in INIT longer before PLAY.");
                telemetry.update();
            }
            return;
        }

        drive.reset();
        for (int i = 0; i < 4; i++) {
            lastSteerTicks[i] = steerAngles[i].getTicks();
        }
        ElapsedTime loopTimer = new ElapsedTime();

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {
            sampleEncoders();

            // Sticks report y-down and x-right; negate for +x forward, +y left, CCW positive.
            drive.driveNormalized(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false);

            double dt = loopTimer.seconds();
            loopTimer.reset();
            checkSteering(dt);

            // Y clears the cut-off once you've fixed the cause.
            if (gamepad1.y) {
                for (int i = 0; i < 4; i++) {
                    steerFault[i] = false;
                    faultTime[i] = 0.0;
                }
            }

            for (int i = 0; i < 4; i++) {
                if (steerFault[i]) {
                    telemetry.addLine("!! " + LABELS[i] + " STEERING OFF: " + faultReason[i]);
                }
            }
            addThroughBoreTelemetry(live, 1.0 / Math.max(dt, 1e-3));
            addSteerEncoderTelemetry();
            telemetry.update();
        }

        drive.stop();
    }

    /** Fresh bulk read, then one sample of every absolute encoder. */
    private void sampleEncoders() {
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
     * Switches a module's steering off when its encoder doesn't follow the
     * motor: pushing hard for FAULT_TIME_S while the count stands still (cable
     * in the wrong port, or unplugged) or runs the wrong way (reversed count).
     * Either one makes the module spin or shake.
     */
    private void checkSteering(double dt) {
        for (int i = 0; i < 4; i++) {
            double ticks = steerAngles[i].getTicks();
            double moved = ticks - lastSteerTicks[i];
            lastSteerTicks[i] = ticks;
            if (steerFault[i]) {
                continue;
            }

            double power = modules[i].getSteerOutput();
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
                          + "'s encoder cable is in ITS OWN motor port. Y to retry."
                        : "encoder counts backwards. Flip STEER_ENCODER_REVERSED[" + i
                          + "]. Y to retry.";
            }
        }
    }

    /**
     * REV Through Bore absolute encoders: the encoder's own angle (averaged, with
     * its +- uncertainty), the wheel angle that gives after direction and offset,
     * and how many samples are in the average. Only accurate while the wheel is
     * still: in INIT it averages everything since INIT, while driving the last
     * ~1 s.
     */
    private void addThroughBoreTelemetry(PwmDutyEstimator[] estimators, double sampleRateHz) {
        telemetry.addLine(String.format("THROUGH BORE  (encoder | wheel | samples)   %.0f samples/s",
                sampleRateHz));
        for (int i = 0; i < 4; i++) {
            telemetry.addData(ENCODER_NAMES[i], "%6.1f° ±%4.1f | %6.1f° | %d",
                    estimators[i].getAngleDegrees(), estimators[i].getStdErrorDegrees(),
                    wheelDegreesFrom(estimators[i], i), estimators[i].getSampleCount());
        }
    }

    /**
     * Steering motors' built-in encoders: port, raw count, counts per second, the
     * wheel angle the steering loop believes, the power being sent, and whether
     * the count moves the way that power should move it. Turn a module by hand
     * in INIT to see which counter moves.
     */
    private void addSteerEncoderTelemetry() {
        telemetry.addLine("STEER MOTOR ENCODERS  (port | count | counts/s | wheel | power | check)");
        for (int i = 0; i < 4; i++) {
            // Checked with the reversal applied, as the steering loop sees it.
            double sign = STEER_ENCODER_REVERSED[i] ? -1.0 : 1.0;
            double velocity = sign * steerMotors[i].getVelocity();
            double power = steerFault[i] ? 0.0 : modules[i].getSteerOutput();
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

    /** Wheel angle in degrees, -180..180, from an encoder estimate. */
    private static double wheelDegreesFrom(PwmDutyEstimator estimator, int i) {
        double deg = ENCODER_DIRECTION * estimator.getAngleDegrees() + ENCODER_OFFSET_DEG[i];
        return Math.toDegrees(SwerveHardware.normalize(Math.toRadians(deg)));
    }
}
