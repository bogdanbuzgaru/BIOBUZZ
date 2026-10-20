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
                    SwerveConstants.STEER_GEAR_RATIO);

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
                    power -> steerMotors[k].setPower(power),
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
                telemetry.addData("sample rate", "%.0f Hz", sampleRateHz);
                for (int i = 0; i < 4; i++) {
                    telemetry.addData(LABELS[i], "encoder %6.1f° ±%4.1f   wheel %6.1f°   n %d",
                            seed[i].getAngleDegrees(), seed[i].getStdErrorDegrees(),
                            wheelDegreesFrom(seed[i], i), seed[i].getSampleCount());
                }
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

            telemetry.addData("loop", "%.1f ms", loopTimer.milliseconds());
            loopTimer.reset();
            for (int i = 0; i < 4; i++) {
                telemetry.addData(LABELS[i],
                        "encoder %6.1f° ±%4.1f | wheel(abs) %6.1f° | wheel(motor) %6.1f° | err %5.1f°",
                        live[i].getAngleDegrees(), live[i].getStdErrorDegrees(),
                        wheelDegreesFrom(live[i], i),
                        Math.toDegrees(steerAngles[i].getAngleRadians()),
                        modules[i].getSteerErrorDegrees());
            }
            telemetry.addLine("encoder / wheel(abs) are averaged over ~1 s and only accurate while the wheel is still.");
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

    /** Wheel angle in degrees, -180..180, from an encoder estimate. */
    private static double wheelDegreesFrom(PwmDutyEstimator estimator, int i) {
        double deg = ENCODER_DIRECTION * estimator.getAngleDegrees() + ENCODER_OFFSET_DEG[i];
        return Math.toDegrees(SwerveHardware.normalize(Math.toRadians(deg)));
    }
}
