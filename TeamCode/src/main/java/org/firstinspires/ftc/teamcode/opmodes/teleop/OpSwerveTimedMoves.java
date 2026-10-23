package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.AbsoluteHome;
import org.firstinspires.ftc.teamcode.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.swerve.SwerveRig;

/**
 * Timed test moves, to measure how far the robot goes. There are no drive
 * encoders, so a move is: point the wheels, ramp the drive power up over
 * RAMP_S, hold CRUISE_POWER for the cruise time, ramp down over RAMP_S. The
 * same move always goes about the same distance; measure it with a tape.
 * Two runs with different cruise times give the speed (difference in
 * distance / difference in time) and the distance covered by the ramps.
 *
 * The Control Hub IMU keeps the heading where it was at the start of the
 * move, and the direction of travel is held relative to that start, so the
 * robot neither turns nor curves. Power is scaled by NOMINAL_VOLTAGE /
 * battery voltage so a full and a tired battery go about the same distance.
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

    // ---------------- moves ----------------

    /** Drive power while cruising, at NOMINAL_VOLTAGE. */
    private static final double CRUISE_POWER = 0.4;
    /** Time to ramp the power up from 0, and down to 0 again, seconds. */
    private static final double RAMP_S = 1.0;
    /** The two cruise times Y switches between, seconds. */
    private static final double[] CRUISE_TIMES_S = {3.0, 6.0};
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

    /** A stick past this cancels a move. */
    private static final double STICK_CANCEL = 0.15;

    private static final double TELEMETRY_INTERVAL_MS = 200.0;

    private enum Phase { IDLE, ALIGN, DRIVE, TURN, SETTLE }

    private SwerveRig rig;
    private SwerveDrive drive;
    private AbsoluteHome absoluteHome;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private Phase phase = Phase.IDLE;
    private double phaseTime = 0.0;
    private int cruiseIndex = 0;

    // the move under way
    private String moveName = "";
    /** Direction of travel relative to the robot at the start, degrees, 0 forward, +90 left. */
    private double moveDirDeg = 0.0;
    private double moveCruiseS = 0.0;
    /** Heading to turn to, degrees, counter-clockwise positive. */
    private double turnTargetDeg = 0.0;
    private double turnHold = 0.0;
    private double drivePower = 0.0;

    private double voltage = NOMINAL_VOLTAGE;
    private double voltageSum = 0.0;
    private int voltageCount = 0;
    private double moveTime = 0.0;
    private String lastResult = "no move yet";

    private boolean prevY, prevUp, prevRight, prevLeft, prevRb, prevLb, prevB, prevX;

    // ---------------------------------------------------------------- INIT

    @Override
    public void init() {
        rig = new SwerveRig(hardwareMap);
        drive = rig.getDrive();
        absoluteHome = new AbsoluteHome(rig);

        telemetry.setAutoClear(false);
        loopTimer.reset();
        telemetryTimer.reset();
        voltageTimer.reset();
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
        phaseTime += dt;
        updateVoltage();

        boolean sticksMoved = Math.abs(gamepad1.left_stick_x) > STICK_CANCEL
                || Math.abs(gamepad1.left_stick_y) > STICK_CANCEL
                || Math.abs(gamepad1.right_stick_x) > STICK_CANCEL;

        if (phase != Phase.IDLE && (gamepad1.a || sticksMoved)) {
            lastResult = "STOPPED by the driver: " + moveName;
            enter(Phase.IDLE);
        }

        readButtons(phase == Phase.IDLE);
        if (phase != Phase.IDLE) {
            moveTime += dt;
            voltageSum += voltage;
            voltageCount++;
        }

        switch (phase) {
            case IDLE:
                // By hand, as in OpSwerve. Sticks report y-down and x-right.
                drive.driveNormalized(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false);
                break;

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
                    finishMove();
                }
                break;
        }

        rig.checkSteering(dt);

        if (telemetryDue()) {
            addTelemetry();
        }
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
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

    // ---------------------------------------------------------------- moves

    private void startDrive(String name, double dirDeg) {
        beginMove(name);
        moveDirDeg = dirDeg;
        moveCruiseS = CRUISE_TIMES_S[cruiseIndex];
        enter(Phase.ALIGN);
    }

    private void startTurn(String name, double headingDeg) {
        beginMove(name);
        turnTargetDeg = headingDeg;
        turnHold = 0.0;
        enter(Phase.TURN);
    }

    private void beginMove(String name) {
        moveName = name;
        rig.resetHeading();
        drive.reset();
        moveTime = 0.0;
        voltageSum = 0.0;
        voltageCount = 0;
        drivePower = 0.0;
    }

    /** Point every wheel the way of travel before any drive power. */
    private void alignWheels(double dt) {
        double dirRad = Math.toRadians(moveDirDeg - rig.getHeadingDegrees());
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
        double total = RAMP_S + moveCruiseS + RAMP_S;
        double profile;
        if (t < RAMP_S) {
            profile = t / RAMP_S;
        } else if (t < RAMP_S + moveCruiseS) {
            profile = 1.0;
        } else {
            profile = Math.max(0.0, (total - t) / RAMP_S);
        }
        drivePower = CRUISE_POWER * profile * NOMINAL_VOLTAGE / voltage;

        double heading = rig.getHeadingDegrees();
        double speed = speedForPower(drivePower);
        // Direction of travel is fixed to the start; turn it into the robot's
        // frame as it is now.
        double dirRad = Math.toRadians(moveDirDeg - heading);
        // Heading correction shrinks with the speed and stops with it. Alone,
        // it would steer the wheels to the turn-on-the-spot pattern (45°)
        // while the robot is still rolling, and it wobbles.
        double omega = 0.0;
        if (speed > 0.0) {
            omega = profile * clamp(HOLD_KP * wrapDeg(0.0 - heading), -HOLD_MAX_RAD_S, HOLD_MAX_RAD_S);
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

        if (turnHold >= TURN_HOLD_S) {
            rig.setDriveCoast(true);
            enter(Phase.SETTLE);
        } else if (phaseTime >= TURN_TIMEOUT_S) {
            // finishMove() shows how far it got
            rig.setDriveCoast(true);
            enter(Phase.SETTLE);
        }
    }

    private void finishMove() {
        double avgVolts = voltageCount == 0 ? voltage : voltageSum / voltageCount;
        double heading = rig.getHeadingDegrees();
        if (moveName.startsWith("turn")) {
            lastResult = String.format("%s: ended at %.1f° (target %.0f°), %.1f s, battery %.2f V",
                    moveName, heading, turnTargetDeg, moveTime, avgVolts);
        } else {
            lastResult = String.format("%s: cruise %.1f s at power %.2f, total %.1f s,"
                            + " heading drift %.1f°, battery %.2f V  -> MEASURE THE DISTANCE",
                    moveName, moveCruiseS, CRUISE_POWER, moveTime, heading, avgVolts);
        }
        enter(Phase.IDLE);
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

    // ---------------------------------------------------------------- helpers

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
            voltage += 0.3 * (sum / n - voltage);
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d > 180.0) {
            d -= 360.0;
        } else if (d < -180.0) {
            d += 360.0;
        }
        return d;
    }

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
        telemetry.addData("cruise time (Y)", "%.0f s", CRUISE_TIMES_S[cruiseIndex]);
        telemetry.addLine("D-pad up fwd | D-pad L/R strafe | bumpers diagonals | X/B turn 45° | A stop");
        switch (phase) {
            case IDLE:
                telemetry.addLine("READY - sticks drive by hand");
                break;
            case ALIGN:
                telemetry.addLine(moveName + ": pointing the wheels");
                break;
            case DRIVE:
                telemetry.addLine(String.format("%s: %.1f s, drive power %.2f", moveName, phaseTime, drivePower));
                break;
            case TURN:
                telemetry.addLine(String.format("%s: %.1f s", moveName, phaseTime));
                break;
            case SETTLE:
                telemetry.addLine(moveName + ": stopping");
                break;
        }
        telemetry.addData("last move", lastResult);
        telemetry.addData("heading", "%.1f°  (IMU %s)", rig.getHeadingDegrees(), rig.getImuStatus());
        telemetry.addData("battery", "%.2f V", voltage);
        rig.addFaultTelemetry(telemetry);
    }
}
