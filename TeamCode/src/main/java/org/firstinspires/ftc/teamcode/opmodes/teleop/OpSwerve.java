package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.AbsoluteHome;
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
 * Driving is robot-centric.
 */
@TeleOp(name = "OpSwerve", group = "drive")
public class OpSwerve extends OpMode {

    /** How often telemetry is rebuilt, so the loop spends its time sampling. */
    private static final double TELEMETRY_INTERVAL_MS = 200.0;

    private SwerveRig rig;
    private SwerveDrive drive;
    private AbsoluteHome absoluteHome;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime rateTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private int samplesSinceRate = 0;
    private double sampleRateHz = 0.0;

    // ---------------------------------------------------------------- INIT

    @Override
    public void init() {
        rig = new SwerveRig(hardwareMap);
        drive = rig.getDrive();
        absoluteHome = new AbsoluteHome(rig);

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
        // Seeded by AbsoluteHome when it finished. If PLAY came first, seed
        // from whatever Through Bore average there is.
        if (!absoluteHome.isDone()) {
            rig.seedFromThroughBores();
        }
        drive.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        rig.sampleEncoders();
        double dt = nextDt();

        // Sticks report y-down and x-right; negate for +x forward, +y left, CCW positive.
        drive.driveNormalized(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);

        rig.checkSteering(dt);

        // Y clears the cut-off once you've fixed the cause.
        if (gamepad1.y) {
            rig.clearSteeringFaults();
        }

        if (telemetryDue()) {
            rig.addFaultTelemetry(telemetry);
            rig.addThroughBoreTelemetry(telemetry, false, sampleRateHz);
            rig.addSteerEncoderTelemetry(telemetry);
        }
    }

    // ---------------------------------------------------------------- STOP

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
    }

    // ----------------------------------------------------------------

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
