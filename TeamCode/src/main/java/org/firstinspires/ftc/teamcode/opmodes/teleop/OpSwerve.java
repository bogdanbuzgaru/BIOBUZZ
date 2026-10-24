package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.AbsoluteHome;
import org.firstinspires.ftc.teamcode.swerve.DriveSequence;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveRig;

/**
 * Swerve test TeleOp.
 *
 * INIT: every wheel turns to its FORWARD position, wherever it was left. The
 * REV Through Bore absolute encoders say where the wheels are and check the
 * result; the steering motor encoders carry out the turn (see AbsoluteHome).
 * FORWARD is the position saved with TRIANGLE in OpSwerveCalibration. Keep
 * the robot still until telemetry says "FORWARD DONE".
 *
 * PLAY: steering runs on each steering motor's built-in encoder, starting
 * from the last Through Bore reading. Left stick drives, right stick turns,
 * Y / TRIANGLE retries a module the steering safety cut-off switched off.
 * The D-pad drives too: up forward, down back, left / right sideways, and two
 * neighbouring buttons together (e.g. up + right) drive on that diagonal.
 * While a D-pad button is held it replaces the left stick.
 *
 * Heading hold: while the robot drives and the right stick is let go, the
 * Control Hub IMU keeps it pointing where it pointed, so it drives straight.
 * Turning with the right stick takes over; the new heading is kept once the
 * stick has been let go for HOLD_DELAY_S. Standing still, nothing is held.
 *
 * Gamepad 2 runs automatic drive sequences: CROSS strafes left then drives a
 * diagonal, SQUARE strafes right then drives a diagonal (45° back-right),
 * TRIANGLE strafes right then turns on the spot (45° left). Their distances
 * and angles are edited on telemetry; see DriveSequence. Driver 1
 * moving a stick or the D-pad stops it at once, as does driver 2's CIRCLE.
 * Driving is robot-centric.
 */
@TeleOp(name = "OpSwerve", group = "drive")
public class OpSwerve extends OpMode {

    /** Driver 1 moving a stick past this stops driver 2's sequence. */
    private static final double DRIVER1_TAKEOVER = 0.15;

    // ---------------- heading hold ----------------

    /** Right stick inside this counts as let go. */
    private static final double TURN_STICK_DEADBAND = 0.05;
    /** After turning, wait this long for the robot to stop rotating before holding the new heading, s. */
    private static final double HOLD_DELAY_S = 0.3;
    /** Turning speed per degree off the held heading, rad/s per degree. */
    private static final double HOLD_KP = 0.04;
    /** Most turning speed the hold uses, rad/s. */
    private static final double HOLD_MAX_RAD_S = 0.5;
    /**
     * Full hold from this much (squared) stick; below it the hold is scaled
     * down, so a robot creeping off from standing doesn't have its wheels
     * steered to the turn-on-the-spot pattern.
     */
    private static final double HOLD_FULL_AT = 0.15;

    /** Drive motor power while driving on the D-pad (0 to 1). */
    private static final double DPAD_POWER = 0.35;

    /** How often telemetry is rebuilt, so the loop spends its time sampling. */
    private static final double TELEMETRY_INTERVAL_MS = 200.0;

    private SwerveRig rig;
    private SwerveDrive drive;
    private AbsoluteHome absoluteHome;
    private DriveSequence sequence;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime rateTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private int samplesSinceRate = 0;
    private double sampleRateHz = 0.0;

    private double heldHeadingDeg = 0.0;
    private boolean holding = false;
    private double sinceTurnS = 0.0;

    // ---------------------------------------------------------------- INIT

    @Override
    public void init() {
        rig = new SwerveRig(hardwareMap);
        drive = rig.getDrive();
        absoluteHome = new AbsoluteHome(rig);
        sequence = new DriveSequence(rig, hardwareMap);

        // The SDK sends telemetry after every cycle. Keep the last screen up
        // between rebuilds instead of building it every cycle.
        telemetry.setAutoClear(false);

        loopTimer.reset();
        rateTimer.reset();
        telemetryTimer.reset();
    }

    @Override
    public void init_loop() {
        rig.sampleEncoders();
        double dt = nextDt();

        // Measure on the Through Bores, turn to forward, check.
        absoluteHome.update(dt);
        if (absoluteHome.isTurning()) {
            // Reads the steering motor encoders, which costs a hub read, so
            // only while they're in use.
            rig.checkSteering(dt);
        }

        if (telemetryDue()) {
            telemetry.addLine("Keep the robot STILL. Wheels turn to FORWARD by the Through Bores.");
            if (!rig.offsetsCameFromFile()) {
                telemetry.addLine("!! No saved FORWARD position. Run OpSwerveCalibration"
                        + " (wheels straight, TRIANGLE, wait for 'captured and saved').");
            }
            absoluteHome.addTelemetry(telemetry);
            rig.addFaultTelemetry(telemetry);
            telemetry.addData("offsets", rig.offsetsAsCode() + "  " + rig.getLoadStatus());
        }
    }

    // ---------------------------------------------------------------- PLAY

    @Override
    public void start() {
        // Seeded by AbsoluteHome when it finished. If PLAY came first, stop it
        // where it is; every steering encoder is still seeded from its wheel's
        // last reading.
        absoluteHome.stop();
        drive.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        rig.sampleEncoders();
        double dt = nextDt();

        // Driver 2's automatic sequence. Driver 1 touching a stick or the
        // D-pad stops it at once and drives by hand from the same loop.
        sequence.update(gamepad2, dt);
        boolean driver1Driving = Math.abs(gamepad1.left_stick_x) > DRIVER1_TAKEOVER
                || Math.abs(gamepad1.left_stick_y) > DRIVER1_TAKEOVER
                || Math.abs(gamepad1.right_stick_x) > DRIVER1_TAKEOVER
                || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
        if (sequence.isRunning() && driver1Driving) {
            sequence.stop("driver 1 took over");
        }

        if (!sequence.isRunning()) {
            // Sticks report y-down and x-right; negate for +x forward, +y left, CCW positive.
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;

            // D-pad: +x forward, +y left. Two neighbouring buttons add up to a
            // diagonal; opposite ones cancel. It asks for DPAD_POWER of top speed
            // and caps the drive motors at DPAD_POWER, so they run at that power.
            // driveNormalized squares each axis, so each one gets the square root
            // of its share: a diagonal is then exactly as fast as a straight move.
            double dpadForward = (gamepad1.dpad_up ? 1.0 : 0.0) - (gamepad1.dpad_down ? 1.0 : 0.0);
            double dpadStrafe = (gamepad1.dpad_left ? 1.0 : 0.0) - (gamepad1.dpad_right ? 1.0 : 0.0);
            double dpadLength = Math.hypot(dpadForward, dpadStrafe);
            if (dpadLength > 0.0) {
                forward = Math.copySign(Math.sqrt(DPAD_POWER * Math.abs(dpadForward) / dpadLength), dpadForward);
                strafe = Math.copySign(Math.sqrt(DPAD_POWER * Math.abs(dpadStrafe) / dpadLength), dpadStrafe);
                rig.setDrivePowerLimit(DPAD_POWER);
            } else {
                rig.setDrivePowerLimit(SwerveRig.MAX_DRIVE_POWER);
            }

            double turnStick = -gamepad1.right_stick_x;
            double translation = Math.hypot(forward * forward, strafe * strafe);
            if (Math.abs(turnStick) > TURN_STICK_DEADBAND || !rig.hasImu()) {
                // Driver turning: no hold, and wait before holding again.
                holding = false;
                sinceTurnS = 0.0;
                drive.driveNormalized(forward, strafe, turnStick, false);
            } else {
                sinceTurnS += dt;
                double heading = rig.getHeadingDegrees();
                if (!holding || translation == 0.0) {
                    // Standing still, or just stopped turning: follow the
                    // heading as it is, and hold it once things have settled.
                    heldHeadingDeg = heading;
                    holding = sinceTurnS >= HOLD_DELAY_S;
                }
                double omega = 0.0;
                if (holding && translation > 0.0) {
                    double error = wrapDeg(heldHeadingDeg - heading);
                    double scale = Math.min(1.0, translation / HOLD_FULL_AT);
                    omega = scale * Math.max(-HOLD_MAX_RAD_S, Math.min(HOLD_MAX_RAD_S, HOLD_KP * error));
                }
                drive.driveNormalizedWithOmega(forward, strafe, omega);
            }
        } else {
            // The sequence steers; start the hold afresh afterwards.
            holding = false;
            sinceTurnS = 0.0;
        }

        rig.checkSteering(dt);

        // Y clears the cut-off once you've fixed the cause.
        if (gamepad1.y) {
            rig.clearSteeringFaults();
        }

        if (telemetryDue()) {
            sequence.addTelemetry(telemetry);
            telemetry.addData("heading hold", !rig.hasImu() ? "OFF (" + rig.getImuStatus() + ")"
                    : holding ? String.format("holding %.1f°, now %.1f°", heldHeadingDeg, rig.getHeadingDegrees())
                    : "off while turning / standing");
            rig.addFaultTelemetry(telemetry);
            rig.addThroughBoreTelemetry(telemetry, false, sampleRateHz);
            rig.addSteerEncoderTelemetry(telemetry);
        }
    }

    // ---------------------------------------------------------------- STOP

    @Override
    public void stop() {
        if (sequence != null) {
            sequence.stop("OpMode stopped");
        }
        if (drive != null) {
            drive.stop();
        }
    }

    // ----------------------------------------------------------------

    private static double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d > 180.0) {
            d -= 360.0;
        } else if (d < -180.0) {
            d += 360.0;
        }
        return d;
    }

    /** Seconds since the last call; also keeps the sample rate up to date. */
    private double nextDt() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        samplesSinceRate++;
        if (rateTimer.seconds() >= 0.5) {
            sampleRateHz = samplesSinceRate / rateTimer.seconds();
            samplesSinceRate = 0;
            rateTimer.reset();
        }
        return dt;
    }

    /** True a few times a second; clears the old screen so a new one can be added. */
    private boolean telemetryDue() {
        if (telemetryTimer.milliseconds() < TELEMETRY_INTERVAL_MS) {
            return false;
        }
        telemetryTimer.reset();
        telemetry.clearAll();
        return true;
    }
}
