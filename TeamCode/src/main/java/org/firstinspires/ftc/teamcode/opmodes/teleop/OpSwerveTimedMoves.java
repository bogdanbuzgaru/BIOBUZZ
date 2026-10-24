package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.AbsoluteHome;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveRig;
import org.firstinspires.ftc.teamcode.swerve.TimedMover;

/**
 * Timed test moves, to measure how far the robot goes (see TimedMover for
 * what a move does). Two runs with different cruise times give the speed
 * (difference in distance / difference in time) and the distance covered by
 * the ramps; those go in TimedMover's calibration table.
 *
 * Every move zeroes the heading first, so each one is measured from the way
 * the robot points when it starts.
 *
 * INIT: wheels turn to FORWARD, as in OpSwerve. Keep the robot still.
 *
 * PLAY, gamepad 1:
 *   Y / TRIANGLE   cruise time 3 s <-> 6 s
 *   D-pad up       forward
 *   D-pad right    strafe right         D-pad left   strafe left
 *   right bumper   diagonal front-right left bumper  diagonal front-left
 *   B / CIRCLE     turn 45° right       X / SQUARE   turn 45° left
 *   A / CROSS      stop the move
 *   sticks         drive by hand between moves; moving a stick also stops a move
 */
@TeleOp(name = "OpSwerveTimedMoves", group = "drive")
public class OpSwerveTimedMoves extends OpMode {

    /** The two cruise times Y switches between, seconds. */
    private static final double[] CRUISE_TIMES_S = {3.0, 6.0};

    /** A stick past this cancels a move. */
    private static final double STICK_CANCEL = 0.15;

    private static final double TELEMETRY_INTERVAL_MS = 200.0;

    private SwerveRig rig;
    private SwerveDrive drive;
    private AbsoluteHome absoluteHome;
    private TimedMover mover;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();

    private int cruiseIndex = 0;
    private String moveName = "";
    private boolean turning = false;
    private String lastResult = "no move yet";

    private boolean prevY, prevUp, prevRight, prevLeft, prevRb, prevLb, prevB, prevX;

    // ---------------------------------------------------------------- INIT

    @Override
    public void init() {
        rig = new SwerveRig(hardwareMap);
        drive = rig.getDrive();
        absoluteHome = new AbsoluteHome(rig);
        mover = new TimedMover(rig, hardwareMap);

        telemetry.setAutoClear(false);
        loopTimer.reset();
        telemetryTimer.reset();
    }

    @Override
    public void init_loop() {
        rig.sampleEncoders();
        double dt = nextDt();

        absoluteHome.update(dt);
        if (absoluteHome.isTurning()) {
            rig.checkSteering(dt);
        }

        if (telemetryDue()) {
            telemetry.addLine("Keep the robot STILL. Wheels turn to FORWARD by the Through Bores.");
            telemetry.addData("IMU", rig.getImuStatus());
            absoluteHome.addTelemetry(telemetry);
            rig.addFaultTelemetry(telemetry);
        }
    }

    // ---------------------------------------------------------------- PLAY

    @Override
    public void start() {
        absoluteHome.stop();
        drive.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        rig.sampleEncoders();
        double dt = nextDt();

        boolean sticksMoved = Math.abs(gamepad1.left_stick_x) > STICK_CANCEL
                || Math.abs(gamepad1.left_stick_y) > STICK_CANCEL
                || Math.abs(gamepad1.right_stick_x) > STICK_CANCEL;

        if (mover.isBusy() && (gamepad1.a || sticksMoved)) {
            mover.stop();
            lastResult = "STOPPED by the driver: " + moveName;
        }

        readButtons(!mover.isBusy());
        mover.update(dt);
        if (mover.consumeFinished()) {
            lastResult = resultText();
        }

        if (!mover.isBusy()) {
            // By hand, as in OpSwerve. Sticks report y-down and x-right.
            drive.driveNormalized(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false);
        }

        rig.checkSteering(dt);

        if (telemetryDue()) {
            addTelemetry();
        }
    }

    @Override
    public void stop() {
        if (mover != null) {
            mover.stop();
        }
    }

    // ---------------------------------------------------------------- buttons

    /** Edge detection runs every loop; a move only starts when idle. */
    private void readButtons(boolean idle) {
        boolean y = gamepad1.y;
        boolean up = gamepad1.dpad_up;
        boolean right = gamepad1.dpad_right;
        boolean left = gamepad1.dpad_left;
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;

        if (!idle) {
            // nothing starts during a move
        } else if (y && !prevY) {
            cruiseIndex = (cruiseIndex + 1) % CRUISE_TIMES_S.length;
        } else if (up && !prevUp) {
            startDrive("forward", 0.0);
        } else if (right && !prevRight) {
            startDrive("strafe right", -90.0);
        } else if (left && !prevLeft) {
            startDrive("strafe left", 90.0);
        } else if (rb && !prevRb) {
            startDrive("diagonal front-right", -45.0);
        } else if (lb && !prevLb) {
            startDrive("diagonal front-left", 45.0);
        } else if (b && !prevB) {
            startTurn("turn 45° right", -45.0);
        } else if (x && !prevX) {
            startTurn("turn 45° left", 45.0);
        }

        prevY = y;
        prevUp = up;
        prevRight = right;
        prevLeft = left;
        prevRb = rb;
        prevLb = lb;
        prevB = b;
        prevX = x;
    }

    private void startDrive(String name, double dirDeg) {
        moveName = name;
        turning = false;
        rig.resetHeading();
        mover.startDrive(dirDeg, CRUISE_TIMES_S[cruiseIndex], 0.0);
    }

    private void startTurn(String name, double headingDeg) {
        moveName = name;
        turning = true;
        rig.resetHeading();
        mover.startTurn(headingDeg);
    }

    private String resultText() {
        double heading = rig.getHeadingDegrees();
        if (turning) {
            return String.format("%s: ended at %.1f° (target %.0f°), %.1f s, battery %.2f V",
                    moveName, heading, mover.getTurnTargetDegrees(), mover.getMoveTime(),
                    mover.getAverageVoltage());
        }
        return String.format("%s: cruise %.1f s at power %.2f, total %.1f s,"
                        + " heading drift %.1f°, battery %.2f V  -> MEASURE THE DISTANCE",
                moveName, mover.getCruiseSeconds(), TimedMover.CRUISE_POWER, mover.getMoveTime(),
                heading, mover.getAverageVoltage());
    }

    // ---------------------------------------------------------------- helpers

    private double nextDt() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        return dt;
    }

    private boolean telemetryDue() {
        if (telemetryTimer.milliseconds() < TELEMETRY_INTERVAL_MS) {
            return false;
        }
        telemetryTimer.reset();
        telemetry.clearAll();
        return true;
    }

    private void addTelemetry() {
        telemetry.addData("cruise time (△ TRIANGLE)", "%.0f s", CRUISE_TIMES_S[cruiseIndex]);
        telemetry.addLine("D-pad ▲ fwd | D-pad ◀ ▶ strafe | L1 / R1 diagonals"
                + " | □ SQUARE / ○ CIRCLE turn 45° left / right | ✕ CROSS stop");
        if (mover.isBusy()) {
            telemetry.addLine(String.format("%s: %s %.1f s, drive power %.2f",
                    moveName, mover.getPhase(), mover.getPhaseTime(), mover.getDrivePower()));
        } else {
            telemetry.addLine("READY - sticks drive by hand");
        }
        telemetry.addData("last move", lastResult);
        telemetry.addData("heading", "%.1f°  (IMU %s)", rig.getHeadingDegrees(), rig.getImuStatus());
        telemetry.addData("battery", "%.2f V", mover.getVoltage());
        rig.addFaultTelemetry(telemetry);
    }
}
